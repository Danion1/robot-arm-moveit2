from setuptools import find_packages, setup

package_name = 'braco2_nos'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel',
    maintainer_email='danion.neros@gmail.com',
    description='Projeto 2 portfólio: nós e cinemática do braço',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mover_para_ponto = braco2_nos.mover_para_ponto:main',
            'pick_and_place = braco2_nos.pick_and_place:main',
        ],
    },
)
