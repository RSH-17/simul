"""Name -> implementation registries for backends and policy kinds."""


class Registry:
    def __init__(self, kind):
        self.kind = kind
        self._items = {}

    def register(self, name):
        def decorate(item):
            if name in self._items:
                raise ValueError(f'{self.kind} {name!r} is already registered')
            self._items[name] = item
            return item
        return decorate

    def get(self, name):
        try:
            return self._items[name]
        except KeyError:
            raise KeyError(f'Unknown {self.kind} {name!r}; known: {", ".join(sorted(self._items))}') from None

    def names(self):
        return sorted(self._items)


BACKENDS = Registry('backend')
POLICY_KINDS = Registry('policy kind')


def load_builtins():
    """Import the built-in implementations so that they register themselves."""
    import osp.backends  # noqa: F401
    import osp.policies  # noqa: F401
