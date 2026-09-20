"""Course priors. A real subpackage so setuptools installs the YAML with it.

Without this file `find_packages` does not see the directory, and the data only
arrives because `package_data` happens to copy it -- setuptools says so out
loud ("Package 'mongla_localization.courses' is absent from the `packages`
configuration"). It worked on the vehicle, which is exactly the kind of luck
that stops working after a setuptools upgrade.
"""
