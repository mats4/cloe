"""
This module contains classes related to the execution of the engine.

This module also contains the PluginSetup class, which encapsulates the
PluginSetup that each Cloe plugin should implement.
"""

import hashlib
import importlib.util
import logging
import os
import re
import shutil
import subprocess
import sys

from pathlib import Path
from collections import OrderedDict
from typing import Dict
from typing import List
from typing import Mapping
from typing import Optional
from typing import Type
from typing import Union

from cloe_launch.utility import run_cmd


class Environment:
    """This class stores a set of environment variables for runtimes.

    The following important variables are required for correct execution:

        PATH

    The main uses are to allow plugins to specify or modify variables that
    they will need when loaded. For example:

        LD_LIBRARY_PATH
        LD_PRELOAD

    Another use is to specify variables that are required for correct
    interpolation of stackfiles:

        CLOE_ROOT
        VTD_ROOT
        IPGHOME

    As one of the primary use-cases of the Environment class is to allow
    modification of the *PATH variables, we provide an extra method for
    modifying path variables.

    Caveats:
    - This is a global namespace, so it's problematic to combine two
      plugins that require different environment variables.
    - Environment variables that contain ":" separated lists do not take
      escaping into account.
    """

    _sep = os.pathsep
    _shell_path: str = "/bin/bash"
    _shell_env: Dict[str, str] = {
        "PATH": "/bin:/usr/bin",
    }
    _preserve: List[str] = [
        # Required for XDG compliance:
        "XDG_.*",
        "HOME",
        "USER",
        # Preserve locale settings:
        "LANGUAGE",
        "LANG",
        "LC_.*",
        # Required for Zsh
        "ZDOTDIR",
        # Required for resolving relative paths:
        "PWD",
        # Required for graphical output:
        "XAUTHORITY",
        "DISPLAY",
        # Required for correct terminal output:
        "COLORTERM",
        "TERM",
        "TERMINAL",
        "S_COLORS",
        # Required for working with proxies:
        "(HTTP|HTTPS|FTP|SOCKS|NO)_PROXY",
        "(http|https|ftp|socks|no)_proxy",
    ]
    _data: Dict[str, str] = {}

    def __init__(
        self,
        env: Union[str, List[Path], Dict[str, str], None],
        preserve: List[str] = None,
        source_file: bool = True,
    ):
        # Initialize shell environment
        if preserve is not None:
            self._preserve = preserve
        self._init_shell_env(os.environ)

        # Initialize the environment
        if isinstance(env, str):
            env = [env]
        if isinstance(env, list):
            self._data = dict()
            for file in env:
                if source_file:
                    self.init_from_shell(file)
                else:
                    self.init_from_file(file)
            if len(env) > 1:
                self.deduplicate_list("PATH")
        elif isinstance(env, dict):
            self.init_from_dict(env)
        else:
            self.init_from_env()

    def _init_shell_env(self, base: Mapping[str, str]) -> None:
        regex = re.compile("^({})$".format("|".join(self._preserve)))
        for key in base:
            if regex.match(key):
                self._shell_env[key] = base[key]

    def init_from_file(self, filepath: Path) -> None:
        """Init variables from a file containing KEY=VALUE pairs."""
        with filepath.open() as file:
            data = file.read()
        self.init_from_str(data)

    def init_from_dict(self, env: Dict[str, str]) -> None:
        """Init variables from a dictionary."""
        self._data = env

    def init_from_shell(self, filepath: Path, shell: str = None) -> None:
        """Init variables from a shell sourcing a file."""
        if shell is None:
            shell = self._shell_path
        cmd = [shell, "-c", f"source {filepath} &>/dev/null && env"]
        result = run_cmd(cmd, env=self._shell_env)
        if result.returncode != 0:
            logging.error(f"Error: error sourcing file from shell: {filepath}")
        self.init_from_str(result.stdout)

    def init_from_env(self) -> None:
        """Init variables from this program's environment."""
        self._data = os.environ.copy()

    def init_from_str(self, data: str) -> None:
        """
        Init variables from a string of KEY=VALUE lines.

        - Leading and trailing whitespace is stripped.
        - Quotes are not removed.
        - Empty lines and lines starting with # are ignored.
        """
        for line in data.split("\n"):
            if line.strip() == "":
                continue
            if line.startswith("#"):
                continue

            kv = line.split("=", 1)
            try:
                self._data[kv[0]] = kv[1]
            except IndexError:
                logging.error(
                    "Error: cannot interpret environment key-value pair: {}".format(
                        line
                    )
                )

    def __delitem__(self, key: str):
        self._data.__delitem__(key)

    def __getitem__(self, key: str) -> str:
        return self._data.__getitem__(key)

    def __setitem__(self, key: str, value: str) -> None:
        self._data.__setitem__(key, value)

    def __str__(self) -> str:
        indent = "    "

        buf = "{\n"
        for k in sorted(self._data.keys()):
            buf += indent + k + ": "
            val = self._data[k]
            if k.endswith("PATH"):
                lst = val.split(self._sep)
                buf += "[\n"
                for path in lst:
                    buf += indent + indent + path + "\n"
                buf += indent + "]"
            else:
                buf += val
            buf += "\n"
        buf += "}"
        return buf

    def path_append(self, key: str, value: Union[Path, str]) -> None:
        """
        Append the value to the path-like variable key.

        This uses ":" as the separator between multiple values in the path.
        """
        if key in self._data:
            self._data[key] += self._sep + str(value)
        else:
            self._data[key] = str(value)

    def path_prepend(self, key: str, value: Union[Path, str]) -> None:
        """
        Prepend the value to the path-like variable key.

        This uses ":" as the separator between multiple values in the path.
        """
        if key in self._data:
            self._data[key] = self._sep + str(value) + self._data[key]
        else:
            self._data[key] = str(value)

    def path_set(self, key: str, values: List[Union[Path, str]]) -> None:
        """Set the key to a :-separated list."""
        self._data[key] = self._sep.join([str(v) for v in values])

    def deduplicate_list(self, key: str) -> None:
        """Remove duplicates from the specified key."""
        if key not in self._data:
            return
        self._data[key] = self._sep.join(
            list(OrderedDict.fromkeys(self._data[key].split(self._sep)))
        )

    def get(self, key: str, default: str = None) -> Optional[str]:
        """Get the value at key or return default."""
        return self._data.get(key, default)

    def get_list(self, key: str, default: List[str] = None) -> Optional[List[str]]:
        """Get the value at key and split or return default."""
        if key in self._data:
            return self._data[key].split(self._sep)
        return default

    def set(self, key: str, value: Union[Path, str]) -> None:
        """Set the value."""
        self._data[key] = str(value)

    def set_default(self, key: str, value: str) -> None:
        """Set the value if it has not already been set."""
        if key not in self._data:
            self._data[key] = value

    def has(self, key: str) -> bool:
        """Return true if the key is in the environment."""
        return key in self._data

    def preserve(self, key: str, override: bool = False):
        """
        Set the given key if it's not already set and it's in the environment.

        When override is True, the value is taken from the environment
        regardless of whether it already exists or not.

        This should not be used for path-like variables.
        """
        if key not in self._data or override:
            value = os.getenv(key, None)
            if value is not None:
                self._data[key] = value

    def export(self, filepath: str) -> None:
        """Write the environment variables to a file in KEY=VALUE pairs."""
        with open(filepath, "w") as file:
            for k in self._data.keys():
                file.write("{}={}\n".format(k, self._data[k]))

    def as_dict(self) -> Dict[str, str]:
        """Return a reference to the internal dictionary."""
        return self._data


class PluginSetup:
    """
    This class loads a file as a Python module and creates from this module
    the PluginSetup class as an embedded type of this class.
    In other words, this class wraps a file as a PluginSetup.

    Requirements:
    - The file must have the '.py' ending.
    - The file must contain a class named 'PluginSetup'.
    - The class PluginSetup must contain the methods:
        extend_env
        setup
        teardown

    In the future, this should be part of the cloe library such that the
    module can base on it in the same way that Conan files base on
    Conan classes. This allows for better type-checking.
    """

    name = None
    plugin = None

    def __init__(self, env: Environment):
        self.env = env

    def environment(self) -> None:
        """
        Modify environment of cloe-engine.

        This can be used to provide values for interpolation in a stackfile
        as well as values required for successful loading and execution of
        the plugin.

        See the Environment class for more details.
        """

    def setup(self) -> None:
        """Perform plugin setup required for simulation."""

    def teardown(self) -> None:
        """Perform plugin teardown after simulation."""


def _find_plugin_setups(file: Path) -> List[Type[PluginSetup]]:
    """Open a Python module and find all PluginSetups."""
    name = os.path.splitext(file)[0]
    spec = importlib.util.spec_from_file_location(name, file)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    # Find all PluginSetup sub-classes. This requires PluginSetup to be
    # imported into the namespace.
    return mod.PluginSetup.__subclasses__()


class Engine:
    """Engine represents the configuration of a single execution of cloe-engine."""

    anonymous_file_regex = "^(/proc/self|/dev)/fd/[0-9]+$"
    engine_path = "cloe-engine"

    def __init__(self, conf, conanfile=None):
        # Set options:
        self.conan_path = Path(conf._conf["conan_path"])
        self.shell_path = Path(conf._conf["shell_path"])
        self.relay_anonymous_files = conf._conf["relay_anonymous_files"]
        if conanfile is None:
            self._read_conf_profile(conf)
        else:
            self._read_anonymous_profile(conanfile)
        self.runtime_dir = Path(conf.profile_runtime(self.profile))
        self.engine_pre_args = conf._conf["engine"]["pre_arguments"]
        self.engine_post_args = conf._conf["engine"]["post_arguments"]
        self.preserve_env = False
        self.conan_args = list()
        self.conan_options = list()
        self.conan_settings = list()
        self.capture_output = True

        logging.info(f"Profile name: {self.profile}")
        logging.info("Configuration:")
        logging.info("   {}".format("\n    ".join(self.profile_data.split("\n"))))

        # Prepare runtime environment
        logging.info(f"Runtime directory: {self.runtime_dir}")

    def _read_conf_profile(self, conf) -> None:
        self.profile = conf.current_profile
        self.profile_path = conf.profile_path(self.profile)
        self.profile_data = conf.read(self.profile)

    def _read_anonymous_profile(self, conanfile) -> None:
        logging.info(f"Source profile: {conanfile}")
        self.profile_path = conanfile
        with open(conanfile) as file:
            self.profile_data = file.read()
        hasher = hashlib.blake2b(digest_size=20)
        hasher.update(self.profile_data.encode())
        self.profile = hasher.hexdigest()

    def runtime_env_path(self) -> Path:
        return self.runtime_dir / "launcher_env.sh"

    def _prepare_runtime_dir(self) -> None:
        """Clean and create runtime directory."""
        self.clean()
        logging.debug(f"Create: {self.runtime_dir}")
        self.runtime_dir.mkdir(parents = True)
        self._write_prompt_sh()
        self._write_bashrc()
        self._write_zshrc()

    def _write_prompt_sh(self) -> None:
        """Write prompt.sh file."""
        prompt_sh_file = self.runtime_dir / "prompt.sh"
        prompt_sh_data = """\
        CLOE_PROMPT="\u001b[2m[cloe-shell]\u001b[0m"

        prompt_cloe() {
            if [[ -n $CLOE_SHELL ]]; then
                PROMPT="%{%F{242}%}[cloe-shell]%f ${PROMPT}"
            fi
        }

        CURRENT_SHELL=$(basename $0)
        if [[ $CURRENT_SHELL = "zsh" ]]; then
            autoload -Uz add-zsh-hook
            add-zsh-hook precmd prompt_cloe
        else
            export PS1="$CLOE_PROMPT $PS1"
        fi
        """
        logging.debug(f"Write: {prompt_sh_file}")
        with prompt_sh_file.open("w") as file:
            file.write(prompt_sh_data)

    def _write_bashrc(self) -> None:
        """Write .bashrc file.

        When creating a new shell, it's not possible to just "source" a file
        and continue with the normal interactive session. So we need to create
        a bashrc file that will be used instead, which loads the normal
        configuration, and then augments it with the extra variables we need.
        """
        bashrc_file = self.runtime_dir / ".bashrc"
        bashrc_data = """\
        source /etc/bash.bashrc
        if [ -f ~/.bashrc ]; then
            source ~/.bashrc
        fi
        OLD_PS1="$PS1"
        source "$(dirname "$BASH_SOURCE[0]")/launcher_env.sh"
        PS1="$OLD_PS1"
        source "$(dirname "$BASH_SOURCE[0]")/prompt.sh"
        """
        logging.debug(f"Write: {bashrc_file}")
        with bashrc_file.open("w") as file:
            file.write(bashrc_data)

    def _write_zshrc(self) -> None:
        """Write .zshrc file."""
        zshrc_file = self.runtime_dir / ".zshrc"
        zshrc_data = f"""\
        ZDOTDIR="${{OLD_ZDOTDIR:-$HOME}}"
        source "${{ZDOTDIR}}/.zshenv"
        source "${{ZDOTDIR}}/.zshrc"
        OLD_PS1="$PS1"
        source "{self.runtime_dir}/launcher_env.sh"
        PS1="$OLD_PS1"
        source "{self.runtime_dir}/prompt.sh"
        """
        logging.debug(f"Write: {zshrc_file}")
        with zshrc_file.open("w") as file:
            file.write(zshrc_data)

    def _prepare_virtualenv(self) -> None:
        # Get conan to create a virtualenv AND virtualrunenv for us:
        # One gives us the LD_LIBRARY_PATH and the other gives us env_info
        # variables set in packages.
        conan_cmd = [
            str(self.conan_path),
            "install",
            "--install-folder",
            str(self.runtime_dir),
            "-g",
            "virtualenv",
            "-g",
            "virtualrunenv",
        ]
        for arg in self.conan_args:
            conan_cmd.append(arg)
        for option in self.conan_options:
            conan_cmd.append("-o")
            conan_cmd.append(option)
        for setting in self.conan_settings:
            conan_cmd.append("-s")
            conan_cmd.append(setting)
        conan_cmd.append(self.profile_path)
        self._run_cmd(conan_cmd, must_succeed=True)

    def _read_conan_env(self) -> Environment:
        # The order of the items in env_paths is important because variables
        # will be overwritten.
        # TODO: Should be replaced by merging in the future.
        env_paths = [
            self.runtime_dir / "activate_run.sh",
            self.runtime_dir / "activate.sh",
        ]
        cloe_launch_env = self.runtime_dir / "cloe_launch_env.sh"
        if cloe_launch_env.exists():
            env_paths.insert(0, cloe_launch_env)
        preserve = None if not self.preserve_env else list(os.environ.keys())
        return Environment(env_paths, preserve=preserve)

    def _extract_engine_path(self, env) -> Path:
        """Return the first cloe-engine we find in the PATH."""
        for bindir in env.get_list("PATH", default=[]):
            pp = Path(bindir) / "cloe-engine"
            if pp.exists():
                return pp
        raise RuntimeError("cannot locate cloe-engine executable")

    def _extract_plugin_paths(self, env) -> List[Path]:
        """Return all Cloe plugin paths we find in LD_LIBRARY_PATH."""
        plugin_paths = []
        for libdir in env.get_list("LD_LIBRARY_PATH", default=[]):
            pp = Path(libdir) / "cloe"
            if pp.exists():
                plugin_paths.append(pp)
        return plugin_paths

    def _extract_plugin_setups(self, lib_paths: List[Path]) -> List[Type[PluginSetup]]:
        for lib_dir in lib_paths:
            for file in lib_dir.iterdir():
                if not file.suffix == ".py":
                    continue
                path = lib_dir / file
                logging.info(f"Loading plugin setup: {path}")
                _find_plugin_setups(path)
        return PluginSetup.__subclasses__()

    def _prepare_runtime_env(self, use_cache: bool = False) -> Environment:
        if self.runtime_env_path().exists() and use_cache:
            logging.debug("Re-using existing runtime directory.")
        else:
            logging.debug("Initializing runtime directory ...")
            self._prepare_runtime_dir()
            self._prepare_virtualenv()

        # Get environment variables we need:
        env = self._read_conan_env()

        # Export Cloe variables:
        env.set("CLOE_PROFILE_HASH", self.profile)
        env.set("CLOE_ENGINE", self._extract_engine_path(env))
        env.path_set("CLOE_PLUGIN_PATH", self._extract_plugin_paths(env))

        return env

    def _write_runtime_env(self, env: Environment) -> None:
        logging.debug(f"Write: {self.runtime_env_path()}")
        env.export(self.runtime_env_path())

    def _process_arg(self, src_path) -> str:
        # FIXME(ben): It may be possible to replace this with the Popen
        # pass_fds argument.

        if not re.match(self.anonymous_file_regex, src_path):
            return src_path

        dst_file = src_path.replace("/", "_")
        dst_path = self.runtime_dir / dst_file
        logging.info(f"Relay anonymous file {src_path} into {dst_path}")
        with open(src_path) as src:
            with open(dst_path, "w") as dst:
                dst.write(src.read())
        return dst_path

    def _engine_cmd(self, args) -> List[str]:
        result = [self.engine_path]
        result.extend(self.engine_pre_args)

        # When invoking cloe-launch with a line like:
        #
        #  cloe-launch exec -- dump <(cat your-file) <(cat next-file)
        #
        # we need to read these files, which are in the format `/proc/self/fd/XYZ`,
        # store them in the runtime directory, and pass them on to the cloe
        # executable.
        if self.relay_anonymous_files:
            result.extend([self._process_arg(a) for a in args])
        else:
            result.extend(args)
        result.extend(self.engine_post_args)
        return result

    def _prepare_plugin_setups(self, env: Environment) -> List[PluginSetup]:
        # Augment environment:
        plugin_paths = self._extract_plugin_paths(env)
        plugin_setup_types = self._extract_plugin_setups(plugin_paths)
        plugin_setups = []
        for setup_type in plugin_setup_types:
            setup = setup_type(env)
            setup.environment()
            plugin_setups.append(setup)

        return plugin_setups

    def clean(self) -> None:
        """Clean the runtime directory."""
        if self.runtime_dir.exists():
            logging.debug(f"Remove: {self.runtime_dir}")
            shutil.rmtree(self.runtime_dir, ignore_errors=True)

    def shell(
        self,
        arguments: List[str] = None,
        use_cache: bool = False,
    ) -> None:
        """Launch a SHELL with the environment variables adjusted."""
        env = self._prepare_runtime_env(use_cache)
        if env.has("CLOE_SHELL"):
            logging.error("Error: recursive cloe shells are not supported.")
            logging.error("Note:")
            logging.error(
                "  It appears you are already in a Cloe shell, since CLOE_SHELL is set."
            )
            logging.error(
                "  Press [Ctrl-D] or run 'exit' to quit this session and then try again."
            )
            sys.exit(2)

        env.set("CLOE_SHELL", self.runtime_env_path())
        self._write_runtime_env(env)

        plugin_setups = self._prepare_plugin_setups(env)
        shell = os.getenv("SHELL", "/bin/bash")

        # Print the final environment, if desired
        logging.debug(f"Environment: {env}")

        if len(plugin_setups) > 0:
            logging.warning("Warning:")
            logging.warning(
                "  The following plugin drivers may contain setup() and teardown()"
            )
            logging.warning("  that will not be called automatically within the shell.")
            logging.warning("")
            for plugin in plugin_setups:
                logging.warning(f"    {plugin.plugin}")

        # Replace this process with the SHELL now.
        sys.stdout.flush()
        cmd = [shell]
        if arguments is not None and len(arguments) != 0:
            cmd.extend(arguments)
        else:
            if shell == "/bin/bash":
                cmd.extend(["--init-file", str(self.runtime_dir/".bashrc")])
            elif shell == "/bin/zsh":
                env.set("OLD_ZDOTDIR", str(env.get("ZDOTDIR", "")))
                env.set("ZDOTDIR", self.runtime_dir)
        logging.debug(f"Exec: {shell} {' '.join(cmd)}")
        os.execvpe(shell, cmd, env.as_dict())

    def activate(
        self,
        use_cache: bool = False,
    ) -> None:
        """Print shell commands to activate a cloe-engine environment."""
        env = self._prepare_runtime_env(use_cache)
        if env.has("CLOE_SHELL"):
            logging.error("Error: recursive cloe shells are not supported.")
            logging.error("Note:")
            logging.error(
                "  It appears you are already in a Cloe shell, since CLOE_SHELL is set."
            )
            logging.error(
                "  Press [Ctrl-D] or run 'exit' to quit this session and then try again."
            )
            sys.exit(2)

        env.set("CLOE_SHELL", self.runtime_env_path())
        self._write_runtime_env(env)

        print("# Please see `cloe-launch activate --help` before activating this.")
        print()
        print(f"source {self.runtime_dir / 'activate.sh'}")
        print(f"source {self.runtime_dir / 'activate_run.sh'}")
        print(f"source {self.runtime_dir / 'prompt.sh'}")
        for var in ["CLOE_PROFILE_HASH", "CLOE_ENGINE", "CLOE_PLUGIN_PATH"]:
            print(f'export {var}="{env[var]}"')
        print(f'export CLOE_SHELL="{self.runtime_env_path()}"')

    def prepare(self, build_policy: str = "outdated") -> None:
        """Prepare (by downloading or building) dependencies for the profile."""
        self.capture_output = False
        if build_policy == "":
            self.conan_args.append("--build")
        else:
            self.conan_args.append(f"--build={build_policy}")
        self._prepare_runtime_env(use_cache=False)

    def exec(
        self,
        args: List[str],
        use_cache: bool = False,
        debug: bool = False,
        override_env: Dict[str, str] = None,
    ) -> subprocess.CompletedProcess:
        """Launch cloe-engine with the environment variables adjusted and with
        plugin setup and teardown."""
        env = self._prepare_runtime_env(use_cache)
        self._write_runtime_env(env)
        plugin_setups = self._prepare_plugin_setups(env)

        # Initialize plugin setups:
        for plugin in plugin_setups:
            logging.debug(
                "Initializing plugin setup for {} at {}".format(
                    plugin.name, plugin.plugin
                )
            )
            plugin.setup()

        # Override environment setup.
        if override_env is not None:
            for k in override_env.keys():
                env[k] = override_env[k]

        # Print the final environment, if desired
        logging.debug(f"Environment: {env}")

        # Run cloe engine:
        self.engine_path = env["CLOE_ENGINE"]
        cmd = self._engine_cmd(args)
        if debug:
            cmd.insert(0, "gdb")
            cmd.insert(1, "--args")
        logging.info("Exec: {}".format(" ".join(cmd)))
        logging.info("---")
        print(end="", flush=True)
        result = subprocess.run(cmd, check=False, env=env.as_dict())
        for setup in plugin_setups:
            setup.teardown()
        return result

    def _run_cmd(self, cmd, must_succeed=True) -> subprocess.CompletedProcess:
        return run_cmd(cmd, must_succeed=must_succeed, capture_output=self.capture_output)
