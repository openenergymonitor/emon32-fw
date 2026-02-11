import subprocess


def extract_commit():
    try:
        commit = subprocess.run(
            ["git", "describe", "--always", "--dirty"],
            capture_output=True,
            check=True,
            text=True,
        ).stdout.strip()
    except subprocess.CalledProcessError:
        commit = "None"

    return commit


def extract_version():
    try:
        release = subprocess.run(
            ["git", "describe", "--always", "--tags", "--abbrev=0"],
            capture_output=True,
            check=True,
            text=True,
        ).stdout.strip()
    except subprocess.CalledProcessError:
        release = "None"

    return release


def main():
    release = extract_version()
    commit = extract_commit()
    print(f"emon32-{release}-{commit}")


if __name__ == "__main__":
    main()
