from setuptools import setup
setup(name='skybeta_flight_interface', version='0.0.0', packages=['skybeta_flight_interface'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/skybeta_flight_interface']),
                  ('share/skybeta_flight_interface', ['package.xml'])],
      entry_points={'console_scripts': ['flight_interface = skybeta_flight_interface.stub:main']})
