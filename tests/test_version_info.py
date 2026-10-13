import subprocess
import sys
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).parent.parent / "scripts"))

import version_info


class VersionInfoTest(unittest.TestCase):
    @mock.patch("version_info.git")
    def test_exact_tag_takes_priority(self, git):
        git.return_value = "v1.2.0"

        self.assertEqual(version_info.extract_version(), "v1.2.0")
        git.assert_called_once_with("describe", "--tags", "--exact-match")

    @mock.patch("version_info.git")
    def test_main_uses_latest_reachable_tag(self, git):
        def result(*args):
            if args == ("describe", "--tags", "--exact-match"):
                raise subprocess.CalledProcessError(128, ["git", *args])
            if args == ("branch", "--show-current"):
                return "main"
            if args == ("describe", "--tags", "--abbrev=0"):
                return "v1.1.1"
            self.fail(f"Unexpected git invocation: {args}")

        git.side_effect = result

        self.assertEqual(version_info.extract_version(), "v1.1.1")

    @mock.patch("version_info.git")
    def test_non_main_uses_highest_release_branch(self, git):
        def result(*args):
            if args == ("describe", "--tags", "--exact-match"):
                raise subprocess.CalledProcessError(128, ["git", *args])
            if args == ("branch", "--show-current"):
                return "features/example"
            if args[0] == "for-each-ref":
                return (
                    "releases/v1.9.0\n"
                    "origin/releases/v1.10.0\n"
                    "origin/releases/v.2.0.0\n"
                    "origin/not-a-release/v9.0.0"
                )
            self.fail(f"Unexpected git invocation: {args}")

        git.side_effect = result

        self.assertEqual(version_info.extract_version(), "v2.0.0b")

    @mock.patch("version_info.git")
    def test_missing_release_branch_returns_none(self, git):
        def result(*args):
            if args == ("branch", "--show-current"):
                return "features/example"
            if args[0] == "for-each-ref":
                return "origin/main"
            raise subprocess.CalledProcessError(128, ["git", *args])

        git.side_effect = result

        self.assertEqual(version_info.extract_version(), "None")


if __name__ == "__main__":
    unittest.main()
