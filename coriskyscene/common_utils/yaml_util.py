import re
import yaml
import os
from collections import OrderedDict

def load_yaml(file, opt=None):
    """
    Load yaml file and return a dictionary.

    Parameters
    ----------
    file : string
        yaml file path.

    opt : argparser
         Argparser.
    
    Returns
    -------
    param : dict
        A dictionary that contains defined parameters.
    """
    if opt and opt.model_dir:
        file = os.path.join(opt.model_dir, 'config.yaml')

    stream = open(file, 'r')
    loader = yaml.Loader
    loader.add_implicit_resolver(
        u'tag:yaml.org,2002:float',
        re.compile(u'''^(?:
         [-+]?(?:[0-9][0-9_]*)\\.[0-9_]*(?:[eE][-+]?[0-9]+)?
        |[-+]?(?:[0-9][0-9_]*)(?:[eE][-+]?[0-9]+)
        |\\.[0-9_]+(?:[eE][-+][0-9]+)?
        |[-+]?[0-9][0-9_]*(?::[0-5]?[0-9])+\\.[0-9_]*
        |[-+]?\\.(?:inf|Inf|INF)
        |\\.(?:nan|NaN|NAN))$''', re.X),
        list(u'-+0123456789.'))
    param = yaml.load(stream, Loader=loader)
    if "yaml_parser" in param:
        param = eval(param["yaml_parser"])(param)

    return param

def save_ordereddict_as_yaml(data, save_path, Dumper=yaml.SafeDumper):
    class OrderedDumper(Dumper):
        pass 

    def _dict_representer(dumper, data):
        return dumper.represent_mapping(
            yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
            data.items()
        )
    
    OrderedDumper.add_representer(OrderedDict, _dict_representer)
    with open(save_path, 'w') as outfile:
        yaml.dump(data, outfile, Dumper=OrderedDumper, default_flow_style=False, sort_keys=False)

def save_yaml(data, save_path):
    """
    Save the dictionary into a yaml file.

    Parameters
    ----------
    data : dict
        The dictionary contains all data.

    save_path : string
        Full path of the output yaml file.
    """
    if isinstance(data, OrderedDict):
        save_ordereddict_as_yaml(data, save_path)
    else:
        with open(save_path, 'w') as outfile:
            yaml.dump(data, outfile, default_flow_style=False, sort_keys=False)


def save_yaml_wo_overwriting(data, save_path):
    """
    Save the yaml file without overwriting the existing one.

    Parameters
    ----------
    data : dict
        The dictionary contains all data.

    save_path : string
        Full path of the output yaml file.
    """
    if os.path.exists(save_path):
        prev_data = load_yaml(save_path)
        data = {**data, **prev_data}

    save_yaml(data, save_path)


def append_to_yaml(data, save_path):
    if not os.path.exists(save_path):
        save_yaml(data, save_path)
    else:
        with open(save_path, 'a') as outfile:
            yaml.dump(data, outfile, default_flow_style=False, sort_keys=False)