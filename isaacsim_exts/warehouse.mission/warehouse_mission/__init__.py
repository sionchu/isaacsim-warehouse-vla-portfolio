try:
    from .extension import WarehouseMissionExtension

    __all__ = ["WarehouseMissionExtension"]
except ModuleNotFoundError as error:
    # Training and unit tests run outside Kit, where the omni modules are absent.
    if error.name != "omni":
        raise
    __all__ = []
