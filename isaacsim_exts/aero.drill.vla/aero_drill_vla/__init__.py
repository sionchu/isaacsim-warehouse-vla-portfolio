try:
    from .extension import AeroDrillVLAExtension

    __all__ = ["AeroDrillVLAExtension"]
except ModuleNotFoundError as error:
    # Training and unit tests run outside Kit, where omni modules are absent.
    if error.name != "omni":
        raise
    __all__ = []
