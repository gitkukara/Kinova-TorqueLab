"""可选用力矩控制算法的控制器注册表"""

import importlib
import inspect
import os
import pkgutil

from .base import BaseController


_SKIP_MODULES = {
    "base",
    "registry",
    "new_controller_template",
}

_CONTROLLER_CACHE = None


def _iter_controller_modules():
    package_name = __package__
    package_dir = os.path.dirname(__file__)
    for module_info in pkgutil.iter_modules([package_dir]):
        module_name = module_info.name
        if module_info.ispkg or module_name in _SKIP_MODULES:
            continue
        yield importlib.import_module(f"{package_name}.{module_name}")


def _iter_controller_classes(module):
    for _, value in inspect.getmembers(module, inspect.isclass):
        if value is BaseController:
            continue
        if not issubclass(value, BaseController):
            continue
        if value.__module__ != module.__name__:
            continue
        name = getattr(value, "name", None)
        if not name or name == BaseController.name:
            continue
        yield name, value


def discover_controllers():
    controllers = {}
    duplicates = {}

    for module in _iter_controller_modules():
        for name, controller_cls in _iter_controller_classes(module):
            if name in controllers:
                duplicates.setdefault(name, [controllers[name]]).append(controller_cls)
                continue
            controllers[name] = controller_cls

    if duplicates:
        details = []
        for name, classes in sorted(duplicates.items()):
            locations = ", ".join(f"{cls.__module__}.{cls.__name__}" for cls in classes)
            details.append(f"{name}: {locations}")
        raise RuntimeError("Duplicate controller names found: " + "; ".join(details))

    return controllers


def controller_classes(refresh=False):
    global _CONTROLLER_CACHE
    if refresh or _CONTROLLER_CACHE is None:
        _CONTROLLER_CACHE = discover_controllers()
    return dict(_CONTROLLER_CACHE)


def available_controller_names():
    return tuple(sorted(controller_classes()))


def get_controller_class(name):
    controllers = controller_classes()
    try:
        return controllers[name]
    except KeyError as exc:
        choices = ", ".join(sorted(controllers))
        raise ValueError(f"Unknown controller '{name}'. Available: {choices}") from exc


def controller_kwargs(controller_cls, config_module, prefix, extra=None):
    """Read matching controller constructor kwargs from config."""
    extra = dict(extra or {})
    signature = inspect.signature(controller_cls.__init__)
    accepted = {
        name
        for name, parameter in signature.parameters.items()
        if name != "self"
        and parameter.kind
        in (
            inspect.Parameter.POSITIONAL_OR_KEYWORD,
            inspect.Parameter.KEYWORD_ONLY,
        )
    }

    kwargs = {}
    config_prefix = prefix.upper() + "_"
    for name in dir(config_module):
        if not name.startswith(config_prefix):
            continue
        key = name[len(config_prefix) :].lower()
        if key in accepted:
            kwargs[key] = getattr(config_module, name)

    for key, value in extra.items():
        if key in accepted:
            kwargs[key] = value
    return kwargs


def create_controller(name, config_module, extra=None):
    controller_cls = get_controller_class(name)
    kwargs = controller_kwargs(controller_cls, config_module, name, extra=extra)
    return controller_cls(**kwargs)
