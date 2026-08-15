# Clearpath Generator Gazebo
import sys
import types

# Compatibility shim for ROS 2 Humble clearpath_config
try:
    import clearpath_config.common.types.exception  # noqa: F401
except (ImportError, ModuleNotFoundError):
    class ClearpathConfigException(Exception):
        """Base exception for clearpath_config."""
        pass

    class UnsupportedAccessoryException(ClearpathConfigException):
        pass

    class UnsupportedMiddlewareException(ClearpathConfigException):
        pass

    class UnsupportedPlatformException(ClearpathConfigException):
        pass

    class UnsupportedSensorException(ClearpathConfigException):
        pass

    class DuplicateAccessoryNameException(ClearpathConfigException):
        pass

    _exc_mod = types.ModuleType("clearpath_config.common.types.exception")
    _exc_mod.ClearpathConfigException = ClearpathConfigException
    _exc_mod.UnsupportedAccessoryException = UnsupportedAccessoryException
    _exc_mod.UnsupportedMiddlewareException = UnsupportedMiddlewareException
    _exc_mod.UnsupportedPlatformException = UnsupportedPlatformException
    _exc_mod.UnsupportedSensorException = UnsupportedSensorException
    _exc_mod.DuplicateAccessoryNameException = DuplicateAccessoryNameException
    sys.modules["clearpath_config.common.types.exception"] = _exc_mod

try:
    from clearpath_config.common.types.platform import Platform
    if not hasattr(Platform, 'A300'):
        Platform.A300 = 'a300'
except Exception:
    pass
