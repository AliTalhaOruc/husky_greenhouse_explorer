from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'frontier_explorer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 1. Launch dosyaları
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        # 2. BURAYI EKLE: config klasöründeki tüm .yaml parametre dosyalarını tanıtır
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
        # 3. İsteğe bağlı: RViz konfigürasyonun veya haritan varsa rviz/map klasörü için:
        # (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ali',
    maintainer_email='ali@todo.todo',
    description='Sera Otonom Keşif Paketi',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'explorer = frontier_explorer.explorer:main',
             'sera_gorev = frontier_explorer.sera_gorev:main',
             'sera_otonom_gorev = frontier_explorer.sera_otonom_gorev:main',
        ],
    },
)
