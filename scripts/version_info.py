import re
import subprocess

RELEASE_BRANCH_PATTERN = re.compile(
    r"(?:^|/)releases/v\.?(?P<major>\d+)\.(?P<minor>\d+)\.(?P<patch>\d+)$"
)


def git(*args):
    return subprocess.run(
        ["git", *args],
        capture_output=True,
        check=True,
        text=True,
    ).stdout.strip()


def try_git(*args):
    try:
        return git(*args)
    except subprocess.CalledProcessError:
        return None


def extract_commit():
    return try_git("describe", "--always", "--dirty") or "None"


def latest_release_version():
    refs = try_git(
        "for-each-ref",
        "--format=%(refname:short)",
        "refs/heads/releases/",
        "refs/remotes/",
    )
    if not refs:
        return None

    versions = []
    for ref in refs.splitlines():
        match = RELEASE_BRANCH_PATTERN.search(ref)
        if match:
            version = tuple(int(match[name]) for name in ("major", "minor", "patch"))
            versions.append(version)

    if not versions:
        return None

    major, minor, patch = max(versions)
    return f"v{major}.{minor}.{patch}b"


def extract_version():
    exact_tag = try_git("describe", "--tags", "--exact-match")
    if exact_tag:
        return exact_tag

    branch = try_git("branch", "--show-current")
    if branch == "main":
        return try_git("describe", "--tags", "--abbrev=0") or "None"

    return latest_release_version() or "None"


def main():
    release = extract_version()
    commit = extract_commit()
    print(f"emon32-{release}-{commit}")


if __name__ == "__main__":
    main()
