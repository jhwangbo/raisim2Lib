from shutil import copyfile
import contextlib
import datetime
import os
import ntpath
import warnings
import torch


@contextlib.contextmanager
def _without_torchscript_deprecation():
    """Silence the TorchScript deprecation notice at call sites that mean it.

    TorchScript is deprecated from Python 3.14 on, but it is still the right
    tool for these policies and it is the only format the C++ policy runner can
    load. Measured on the 34->128->128->12 ANYmal policy, batch of 100, single
    threaded: TorchScript 84 us/call, AOTInductor 86, eager 93, torch.compile
    128. Producing the artifact costs 0.01 s with TorchScript against 5 s warm
    and 19 s cold for torch.export plus AOTInductor, and the evaluation policy
    is re-exported on every evaluation.

    So neither suggested replacement is an improvement today. AOTInductor is the
    one to switch to when TorchScript stops working: it matches TorchScript's
    speed, and its export cost could be amortized by caching the package and
    swapping weights through update_constant_buffer. Until then every caller
    here degrades to eager or to the Python evaluation loop on failure.
    """
    with warnings.catch_warnings():
        warnings.filterwarnings('ignore', category=FutureWarning, module=r'torch\.jit.*')
        yield


def script_policy(module):
    """TorchScript-compile an inference-only policy."""
    with _without_torchscript_deprecation():
        return torch.jit.script(module)


def save_scripted_policy(scripted_module, path):
    """Serialize a scripted policy for the C++ policy runner."""
    with _without_torchscript_deprecation():
        torch.jit.save(scripted_module, path)


class ConfigurationSaver:
    def __init__(self, log_dir, save_items):
        self._data_dir = log_dir + '/' + datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        os.makedirs(self._data_dir)

        if save_items is not None:
            for save_item in save_items:
                base_file_name = ntpath.basename(save_item)
                copyfile(save_item, self._data_dir + '/' + base_file_name)

    @property
    def data_dir(self):
        return self._data_dir
        

def tensorboard_launcher(directory_path):
    from tensorboard import program
    import webbrowser
    # learning visualizer
    tb = program.TensorBoard()
    tb.configure(argv=[None, '--logdir', directory_path])
    url = tb.launch()
    print("[RAISIM_GYM] Tensorboard session created: "+url)
    webbrowser.open_new(url)


def load_param(weight_path, env, actor, critic, optimizer, data_dir):
    if weight_path == "":
        raise Exception("\nCan't find the pre-trained weight, please provide a pre-trained weight with --weight switch\n")
    print("\nRetraining from the checkpoint:", weight_path+"\n")

    iteration_number = weight_path.rsplit('/', 1)[1].split('_', 1)[1].rsplit('.', 1)[0]
    weight_dir = weight_path.rsplit('/', 1)[0] + '/'

    mean_csv_path = weight_dir + 'mean' + iteration_number + '.csv'
    var_csv_path = weight_dir + 'var' + iteration_number + '.csv'
    items_to_save = [weight_path, mean_csv_path, var_csv_path, weight_dir + "cfg.yaml", weight_dir + "Environment.hpp"]

    if items_to_save is not None:
        pretrained_data_dir = data_dir + '/pretrained_' + weight_path.rsplit('/', 1)[0].rsplit('/', 1)[1]
        os.makedirs(pretrained_data_dir)
        for item_to_save in items_to_save:
            copyfile(item_to_save, pretrained_data_dir+'/'+item_to_save.rsplit('/', 1)[1])

    # load observation scaling from files of pre-trained model
    env.load_scaling(weight_dir, iteration_number)

    # load actor and critic parameters from full checkpoint
    checkpoint = torch.load(weight_path)
    actor.architecture.load_state_dict(checkpoint['actor_architecture_state_dict'])
    actor.distribution.load_state_dict(checkpoint['actor_distribution_state_dict'])
    critic.architecture.load_state_dict(checkpoint['critic_architecture_state_dict'])
    optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
