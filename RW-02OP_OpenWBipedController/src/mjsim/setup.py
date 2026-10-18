from setuptools import find_packages, setup
import glob, os

package_name = 'mjsim'

# 递归收集models目录下的所有文件（包括子目录）
model_files = []
models_dir = 'models'

for root, dirs, files in os.walk(models_dir):
    install_dir = os.path.join('share', package_name, root)
    source_files = [os.path.join(root, filename) for filename in files]
    if source_files:
        model_files.append((install_dir, source_files))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'mjsim.web_monitor': ['static/*', 'static/**/*'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*')),
        ('share/' + package_name + '/config/motions', glob.glob('config/motions/*.json')),
    ] + model_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shiro',
    maintainer_email='hitcw@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sim_node = mjsim.sim_node:main',
            'panda_controller = mjsim.controller.panda_controller:main',
            'panda_controller2 = mjsim.controller.panda_controller2:main',
            'model_viewer = mjsim.viewer.model_viewer:main',
            'viser_model_viewer = mjsim.viewer.viser_model_viewer:main',
            'log_node = mjsim.controller.log_node:main',
            'kb_joy = mjsim.controller.kb_joy:main',
            'web_monitor = mjsim.web_monitor.monitor_node:main',
        ],
    },
)
