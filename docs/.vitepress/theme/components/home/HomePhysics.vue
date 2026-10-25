<script setup lang="ts">
import { withBase } from 'vitepress'
import LazyVideo from '../LazyVideo.vue'
import { clips } from '../../media'

const cards = [
  {
    clip: clips.suspension,
    title: 'Articulated vehicles',
    text: 'Suspension, steering and drive shafts move as one articulated system, with live spring and damper forces.',
    link: '/sections/ArticulatedSystem.html',
  },
  {
    clip: clips.soil,
    title: 'Deformable terrain',
    text: 'Tires sink and leave lasting ruts, with the whole contact patch resolved, for off-road and planetary rovers.',
    link: '/sections/DeformableTerrain.html',
  },
  {
    clip: clips.strandbeest,
    title: 'Closed kinematic loops',
    text: 'Linkage-driven legs and parallel mechanisms: a Strandbeest walks on 36 closed loops from a single crank.',
    link: '/sections/Constraints.html',
  },
  {
    clip: clips.hand,
    title: 'Dexterous contact',
    text: 'A Shadow Hand from OpenUSD grasps a cube, with many simultaneous contacts for in-hand manipulation.',
    link: '/sections/OpenUSD.html',
  },
  {
    clip: clips.usd,
    title: 'Bring your robots',
    text: 'URDF, MJCF and OpenUSD models load directly, with their meshes and materials.',
    link: '/sections/articulated_system/Creation.html',
  },
  {
    clip: clips.tracks,
    title: 'Tracked vehicles',
    text: 'Tracks are simulated as driven belts, so a flipper robot crosses loose rubble and climbs stairs on contact friction alone.',
    link: '/sections/articulated_system/Tracks.html',
  },
]
</script>

<template>
  <section id="physics" class="rs-section rs-physics">
    <div class="rs-container">
      <p v-reveal class="rs-eyebrow">Physics</p>
      <h2 v-reveal="60" class="rs-h2">Physics that holds up on real robots.</h2>
      <p v-reveal="120" class="rs-lead">
        Legged-robot controllers trained in RaiSim have walked real robots over rough terrain in
        <a class="rs-inline-link" :href="withBase('/sections/HallOfFame.html')">published research</a>. Hard contacts
        with friction, closed loops, tracks, deformable soil and granular media are solved together, and the same inputs
        give the same trajectory, run after run.
      </p>

      <div class="rs-physics__grid">
        <a
          v-for="(card, index) in cards"
          :key="card.title"
          v-reveal="(index % 3) * 90"
          v-glow
          class="rs-card rs-physics__card"
          :href="withBase(card.link)"
        >
          <div class="rs-physics__media">
            <LazyVideo :src="card.clip.src" :poster="card.clip.poster" :label="card.title" />
          </div>
          <div class="rs-physics__body">
            <h3>{{ card.title }}</h3>
            <p>{{ card.text }}</p>
            <span class="rs-physics__more" aria-hidden="true">
              Learn more
              <svg viewBox="0 0 16 16"><path d="M3 8h9M8.5 4l4 4-4 4" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" /></svg>
            </span>
          </div>
        </a>
      </div>
    </div>
  </section>
</template>

<style scoped>
.rs-physics__grid {
  display: grid;
  grid-template-columns: repeat(3, minmax(0, 1fr));
  gap: 18px;
  margin-top: 56px;
}

.rs-physics__card {
  display: flex;
  flex-direction: column;
  overflow: hidden;
  color: inherit !important;
  text-decoration: none !important;
}

@media (hover: hover) {
  .rs-physics__card:hover {
    transform: translateY(-6px);
  }

  .rs-physics__card:hover .rs-physics__media :deep(video) {
    transform: scale(1.06);
  }

  .rs-physics__card:hover .rs-physics__more {
    color: var(--rs-text);
  }

  .rs-physics__card:hover .rs-physics__more svg {
    transform: translateX(4px);
  }
}

.rs-physics__media {
  position: relative;
  aspect-ratio: 16 / 10;
  overflow: hidden;
  background: #000;
}

.rs-physics__media :deep(video) {
  transition: transform 1.2s var(--rs-ease);
}

.rs-physics__body {
  display: flex;
  flex: 1;
  flex-direction: column;
  padding: 22px 24px 24px;
}

.rs-physics__body h3 {
  margin: 0;
  font-size: 20px;
  font-weight: 700;
  letter-spacing: -0.02em;
  color: var(--rs-text);
}

.rs-physics__body p {
  flex: 1;
  margin: 8px 0 18px;
  font-size: 15px;
  line-height: 1.6;
  color: var(--rs-text-2);
}

.rs-physics__more {
  display: inline-flex;
  align-items: center;
  gap: 8px;
  font-size: 14px;
  font-weight: 650;
  color: var(--rs-text-3);
  transition: color 0.25s ease;
}

.rs-physics__more svg {
  width: 15px;
  height: 15px;
  transition: transform 0.25s var(--rs-ease);
}

@media (max-width: 1023px) {
  .rs-physics__grid {
    grid-template-columns: repeat(2, minmax(0, 1fr));
  }
}

@media (max-width: 600px) {
  .rs-physics__grid {
    grid-template-columns: minmax(0, 1fr);
    gap: 14px;
    margin-top: 40px;
  }

  .rs-physics__body {
    padding: 18px 18px 20px;
  }
}
</style>
