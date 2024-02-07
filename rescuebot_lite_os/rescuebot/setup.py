from setuptools import setup, find_packages

setup(
    name='rescuebot',
    version='0.1.0',
    author='Donia E.',
    author_email='donia.e@example.com',
    description='Installs rescute bot OS',
    packages=find_packages(),
    install_requires=[
        # List your project dependencies here.
        # For example:
        'requests',
        'kombu',
    ],
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'Natural Language :: English',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3.12',
    ],
    python_requires='>=3.12'
)