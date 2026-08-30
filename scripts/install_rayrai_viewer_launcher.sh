#!/usr/bin/env bash
#
# Install a desktop launcher for rayrai_tcp_viewer and pin it to the GNOME /
# Ubuntu dock, so the viewer is one click away instead of a terminal command.
#
# Run it directly after a build, or let CMake run it: configuring the examples
# with -DRAISIM_EXAMPLE_DESKTOP_LAUNCHER=ON adds a post-build step that invokes
# this script on every Release build of the rayrai_tcp_viewer target.
#
#   scripts/install_rayrai_viewer_launcher.sh                 # install + pin
#   scripts/install_rayrai_viewer_launcher.sh --no-pin        # install only
#   scripts/install_rayrai_viewer_launcher.sh --uninstall     # remove everything
#
# Everything it writes lives under the invoking user's ~/.local; no root, no
# system-wide state. Re-running is idempotent.
#
# Options:
#   --viewer PATH   Executable to launch (default: <repo>/build-examples/examples/rayrai_tcp_viewer)
#   --repo PATH     Repository root, used for raisim_env.sh and the icon
#                   (default: the directory containing this script's parent)
#   --config CFG    Build configuration; anything other than Release is skipped.
#                   Omit to install regardless of configuration.
#   --icon-shape S  Icon plate shape: rounded (default), circle or square
#   --icon-background COLOR
#                   Plate fill, any ImageMagick colour (default: #dedede)
#   --pin/--no-pin  Whether to add the launcher to the dock favourites (default: pin)
#   --uninstall     Remove the launcher, wrapper, icon and dock entry
#   --quiet         Only report failures

set -eo pipefail

APP_ID=rayrai-tcp-viewer
DESKTOP_DIR="$HOME/.local/share/applications"
DESKTOP_FILE="$DESKTOP_DIR/$APP_ID.desktop"
WRAPPER="$HOME/.local/bin/$APP_ID"
ICON_THEME_DIR="$HOME/.local/share/icons/hicolor"
ICON_FALLBACK="$HOME/.local/share/rayrai/$APP_ID.png"
ICON_SIZES="48 64 128 256 512"
# Shape and fill of the icon plate the logo is drawn on. The plate is a light
# grey rather than white: pure white reads as a hard slab in the dock, and the
# logo's own white ribbon needs something to separate from.
ICON_SHAPE=rounded
ICON_BACKGROUND="#dedede"

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO="${script_dir%/*}"
VIEWER=""
CONFIG=""
PIN=1
UNINSTALL=0
QUIET=0

while [ $# -gt 0 ]; do
  case "$1" in
    --viewer) VIEWER="$2"; shift ;;
    --repo) REPO="$2"; shift ;;
    --config) CONFIG="$2"; shift ;;
    --icon-shape) ICON_SHAPE="$2"; shift ;;
    --icon-background) ICON_BACKGROUND="$2"; shift ;;
    --pin) PIN=1 ;;
    --no-pin) PIN=0 ;;
    --uninstall) UNINSTALL=1 ;;
    --quiet) QUIET=1 ;;
    -h|--help) sed -n '2,28p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "install_rayrai_viewer_launcher: unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done

say() { [ "$QUIET" = 1 ] || echo "$@"; }

# Rewrite the dock favourites list, preserving the user's other entries.
set_favorites() {
  local action="$1" current
  command -v gsettings >/dev/null 2>&1 || return 0
  current=$(gsettings get org.gnome.shell favorite-apps 2>/dev/null) || return 0
  local updated
  updated=$(python3 - "$current" "$APP_ID.desktop" "$action" <<'PY'
import ast, sys
try:
    apps = ast.literal_eval(sys.argv[1])
except (ValueError, SyntaxError):
    sys.exit(1)
entry, action = sys.argv[2], sys.argv[3]
if action == "add":
    if entry not in apps:
        apps.append(entry)
else:
    apps = [a for a in apps if a != entry]
print(repr(apps))
PY
  ) || return 0
  [ "$updated" = "$current" ] && return 0
  gsettings set org.gnome.shell favorite-apps "$updated" || return 0
  say "install_rayrai_viewer_launcher: dock favourites updated"
}

# Install the RaiSim logo as the launcher icon. Icon themes match a PNG to the
# directory it sits in, so the source logo (321x322) has to be resized to each
# exact size rather than dropped into 256x256/apps as-is -- a mismatch leaves the
# dock showing a generic placeholder. Without ImageMagick we fall back to an
# absolute Icon= path, which GNOME scales itself.
install_icon() {
  local logo="$REPO/docs/image/logo.png" magick=""
  [ -f "$logo" ] || { echo "install_rayrai_viewer_launcher: no logo at $logo" >&2; return 1; }
  if command -v magick >/dev/null 2>&1; then magick=magick
  elif command -v convert >/dev/null 2>&1; then magick=convert
  fi

  if [ -z "$magick" ]; then
    mkdir -p "$(dirname "$ICON_FALLBACK")"
    cp -f "$logo" "$ICON_FALLBACK"
    ICON_NAME="$ICON_FALLBACK"
    return 0
  fi

  local size dir plate art scale shape_draw radius tmp
  tmp=$(mktemp -d)
  for size in $ICON_SIZES; do
    dir="$ICON_THEME_DIR/${size}x${size}/apps"
    mkdir -p "$dir"
    plate="$tmp/plate.png"
    art="$tmp/art.png"

    # The logo's lower third is a transparent wordmark, so drawing it straight
    # onto the canvas leaves the coloured mark sitting high and hard-edged. Draw
    # it on an opaque plate instead: that gives the icon a centred, rounded
    # silhouette, and keeps the dark wordmark readable on a dark dock.
    case "$ICON_SHAPE" in
      circle)
        shape_draw="circle $((size / 2)),$((size / 2)) $((size / 2)),0"
        scale=66
        ;;
      square)
        shape_draw="rectangle 0,0,$((size - 1)),$((size - 1))"
        scale=80
        ;;
      *)
        radius=$((size * 22 / 100))
        shape_draw="roundrectangle 0,0,$((size - 1)),$((size - 1)),$radius,$radius"
        scale=80
        ;;
    esac

    # PNG32 / -colorspace sRGB matter: a plate drawn in a grey or white on
    # transparent is stored as greyscale otherwise, and compositing onto it
    # drains the colour out of the logo.
    "$magick" -size "${size}x${size}" xc:none -colorspace sRGB \
      -fill "$ICON_BACKGROUND" -draw "$shape_draw" "PNG32:$plate"
    "$magick" "$logo" -resize "$((size * scale / 100))x$((size * scale / 100))" \
      -background none -gravity center -extent "${size}x${size}" "PNG32:$art"
    "$magick" "$plate" "$art" -composite "PNG32:$dir/$APP_ID.png"
  done
  rm -rf "$tmp"

  # A user-local hicolor tree often has no index.theme, and gtk-update-icon-cache
  # refuses to run without one. Only create it if missing, and describe exactly
  # the directories written above; icon lookup itself merges this with the system
  # hicolor theme, so existing icons in this tree keep resolving.
  if [ ! -f "$ICON_THEME_DIR/index.theme" ]; then
    {
      printf '[Icon Theme]\nName=Hicolor\nComment=Fallback icon theme\nHidden=true\n'
      printf 'Directories='
      local sep=""
      for size in $ICON_SIZES; do printf '%s%sx%s/apps' "$sep" "$size" "$size"; sep=","; done
      printf '\n'
      for size in $ICON_SIZES; do
        printf '\n[%sx%s/apps]\nSize=%s\nContext=Applications\nType=Threshold\n' \
          "$size" "$size" "$size"
      done
    } > "$ICON_THEME_DIR/index.theme"
  fi
  command -v gtk-update-icon-cache >/dev/null 2>&1 && \
    gtk-update-icon-cache -q -f -t "$ICON_THEME_DIR" 2>/dev/null || true
  ICON_NAME="$APP_ID"
}

remove_icons() {
  local size
  for size in $ICON_SIZES; do
    rm -f "$ICON_THEME_DIR/${size}x${size}/apps/$APP_ID.png"
  done
  rm -f "$ICON_FALLBACK"
  command -v gtk-update-icon-cache >/dev/null 2>&1 && \
    gtk-update-icon-cache -q -f -t "$ICON_THEME_DIR" 2>/dev/null || true
}

if [ "$UNINSTALL" = 1 ]; then
  set_favorites remove
  remove_icons
  rm -f "$DESKTOP_FILE" "$WRAPPER"
  command -v update-desktop-database >/dev/null 2>&1 && \
    update-desktop-database "$DESKTOP_DIR" 2>/dev/null || true
  say "install_rayrai_viewer_launcher: removed launcher, wrapper, icon and dock entry"
  exit 0
fi

# --- Conditions under which we quietly do nothing -----------------------------
# These all run inside a build, so none of them may fail the build.

if [ -n "$CONFIG" ] && [ "$CONFIG" != Release ]; then
  say "install_rayrai_viewer_launcher: skipped ($CONFIG build, launcher is Release-only)"
  exit 0
fi

case "$(uname -s)" in
  Linux) ;;
  *) say "install_rayrai_viewer_launcher: skipped (not Linux)"; exit 0 ;;
esac

# A headless build machine has no dock to pin to, and writing into a CI home
# directory is pointless noise.
if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
  say "install_rayrai_viewer_launcher: skipped (no graphical session)"
  exit 0
fi

[ -n "$VIEWER" ] || VIEWER="$REPO/build-examples/examples/rayrai_tcp_viewer"
if [ ! -x "$VIEWER" ]; then
  echo "install_rayrai_viewer_launcher: no executable at $VIEWER; skipping" >&2
  exit 0
fi
if [ ! -f "$REPO/raisim_env.sh" ]; then
  echo "install_rayrai_viewer_launcher: no raisim_env.sh under $REPO; skipping" >&2
  exit 0
fi

# --- Install ------------------------------------------------------------------

mkdir -p "$DESKTOP_DIR" "$(dirname "$WRAPPER")"

# The dock starts applications with a login-shell environment that has no
# LD_LIBRARY_PATH entries for raisim/lib and rayrai/lib, so a bare Exec= line
# would launch the viewer and have it die on the first missing shared object.
# The wrapper sources raisim_env.sh the way a terminal user would.
cat > "$WRAPPER" <<WRAP
#!/usr/bin/env bash
# Generated by scripts/install_rayrai_viewer_launcher.sh -- edits will be lost.
set -e
source "$REPO/raisim_env.sh"
export RAISIM_LOCAL_INSTALL_ROOT="$REPO"
# librayrai reads the activation key from the relative path ".raisim" rather
# than \$HOME/.raisim. Started from a directory that holds a .raisim *directory*
# -- \$HOME, which is exactly the working directory a dock launch inherits --
# the read throws "Is a directory" and the process aborts on startup. Move out
# of such a directory first; relative paths on the command line are otherwise
# left alone, and this becomes a no-op once the library resolves \$HOME itself.
if [ -d ".raisim" ]; then
  cd "$(dirname "$VIEWER")"
fi
# Match StartupWMClass in the .desktop file so the running window groups under
# the pinned icon instead of showing up as a separate dock entry.
export SDL_VIDEO_X11_WMCLASS=$APP_ID
export SDL_VIDEO_WAYLAND_WMCLASS=$APP_ID
exec "$VIEWER" "\$@"
WRAP
chmod +x "$WRAPPER"

ICON_NAME="$APP_ID"
install_icon || ICON_NAME="applications-science"

cat > "$DESKTOP_FILE" <<DESK
[Desktop Entry]
Type=Application
Version=1.4
Name=Rayrai TCP Viewer
GenericName=RaiSim Visualizer
Comment=Connect to a running RaisimServer simulation and render it with rayrai
Exec=$WRAPPER %U
Path=$(dirname "$VIEWER")
Icon=$ICON_NAME
Terminal=false
Categories=Science;Engineering;
Keywords=raisim;rayrai;simulation;robotics;viewer;
StartupNotify=true
StartupWMClass=$APP_ID
DESK

command -v update-desktop-database >/dev/null 2>&1 && \
  update-desktop-database "$DESKTOP_DIR" 2>/dev/null || true

say "install_rayrai_viewer_launcher: installed $DESKTOP_FILE -> $VIEWER"
[ "$PIN" = 1 ] && set_favorites add
exit 0
