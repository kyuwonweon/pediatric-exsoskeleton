####################################################################################################
# @file
#
# @brief            Library module for defining custom Python decorators.
#
# @author           Michael Stephens
####################################################################################################


####################################################################################################
# @brief            Use as a decorator to utilize a singleton pattern.
#
# @details          Supports mutliple 'instances' where an instance is a unique class (multiple
#                   classes can use this decorator to make use of a singleton pattern individually).
#
# @param[in]        cls                 Class to generate instance of or return existing instance.
# @param[in]        *args               Auto-filled in with all non-keyword arguments [e.g. a, b].
# @param[in]        **kw                Auto-filled inwith all keyword arguments [e.g. a=1, b=2].
#
# @note             To utilize this decorator:
#                       * Internal to this library:
#                           * from . import decorators
#                           * @decorators.singleton
#                           * class Foo(): ...
#                       * External to this library:
#                           * import ll_common
#                           * @ll_common.decorators.singleton
#                           * class Foo(): ...
####################################################################################################

print('IMPORTING DECORATORS')


def singleton(cls, *args, **kw):

    instances = {}

    ################################################################################################
    # @brief        Inner method used to implement the main decorator functionality.
    ################################################################################################
    def _singleton():
        # If class is not alaready in the list of instances, initialize it.
        if cls not in instances:
            instances[cls] = cls(*args, **kw)

        # Return the tracked class instance to caller.
        return instances[cls]

    # Return call to inner function. This invoked funtion handles creating the first instance or
    # returning the existing one.
    return _singleton
