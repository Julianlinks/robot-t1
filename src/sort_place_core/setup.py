from setuptools import setup

setup(name='sort_place_core', version='0.1.0', packages=['sort_place_core'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/sort_place_core']),
                  ('share/sort_place_core', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Cell Maintainer', maintainer_email='maintainer@example.com',
      description='SCARA sorting and placement task engine', license='Apache-2.0',
      entry_points={'console_scripts': [
          'task_server = sort_place_core.ros_node:main',
          'offline_demo = sort_place_core.offline_demo:main']})
