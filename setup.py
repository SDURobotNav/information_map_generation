from distutils.core import setup
 
setup(
    version='0.0.0',
    scripts=['scripts/info_map_gen.py',
            'scripts/network.py',
            'scripts/nn_interface.py',
            'scripts/raw_map_proc.py',
            'scripts/visualization.py'],
    packages=['information_map'],
    package_dir={'': 'scripts'}
)