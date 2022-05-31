# Frontier Exploration with a prior

## Dependency

- Python 3.9

### Check your Python version

```sh
python --version
```

---

## Install project as package

```sh
python -m pip install --no-cache git+https://github.com/ekumenlabs/Frontier-Exploration-with-a-prior.git@master
```

---

## Install for local dev

### Create virtualenv

```sh
python -m pip install --upgrade pip venv
python -m venv ./venv
```

### Activate virtualenv

```sh
source ./venv/bin/activate
```

### Install dependencies and local project

```sh
python -m pip install -r requirements.txt
python -m pip install -e .
```

## When you're done:

```sh
deactivate
```

## Dev workflow

```sh
source dev.sh
```

---
