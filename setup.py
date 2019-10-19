import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

# What license are we?
setuptools.setup(
    name="raas_envs",
    version="0.0.1",
    author="Max King, Benjamin Wiener, Declan Oller, Philip Zucker",
    author_email="team@perciplex.com",
    description="Custom OpenAIgym environments for Perciplex RaaS platform",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/perciplex/raas-env",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.6",
)
