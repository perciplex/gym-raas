import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

# What license are we?
setuptools.setup(
    name="gym_raas",
    version="0.0.1",
    author="Max King, Benjamin Wiener, Declan Oller, Philip Zucker",
    author_email="team@perciplex.com",
    description="Custom OpenAI gym environments for Perciplex RaaS platform",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/perciplex/gym-raas",
    packages=setuptools.find_packages(),
    install_requires=["gym"],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.6",
)
