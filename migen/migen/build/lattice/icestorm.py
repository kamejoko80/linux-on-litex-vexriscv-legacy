# This file is Copyright (c) 2016-2017 William D. Jones <thor0505@comcast.net>
# License: BSD

import os
import sys
import subprocess

from migen.fhdl.structure import _Fragment

from migen.build.generic_platform import *
from migen.build import tools
from migen.build.lattice import common


def _build_pcf(named_sc, named_pc):
    r = ""
    for sig, pins, others, resname in named_sc:
        if len(pins) > 1:
            for bit, pin in enumerate(pins):
                r += "set_io {}[{}] {}\n".format(sig, bit, pin)
        else:
            r += "set_io {} {}\n".format(sig, pins[0])
    if named_pc:
        r += "\n" + "\n\n".join(named_pc)
    return r


def _build_pre_pack(vns, freq_cstrs):
    r = ""
    for sig in freq_cstrs:
        r += """ctx.addClock("{}", {})\n""".format(vns.get_name(sig), freq_cstrs[sig])
    return r


def _build_script(source, build_template, build_name, **kwargs):
    if sys.platform in ("win32", "cygwin"):
        script_ext = ".bat"
        build_script_contents = "@echo off\nrem Autogenerated by Migen\n\n"
        fail_stmt = " || exit /b"
    else:
        script_ext = ".sh"
        build_script_contents = "# Autogenerated by Migen\nset -e\n\n"
        fail_stmt = ""

    for s in build_template:
        s_fail = s + "{fail_stmt}\n"  # Required so Windows scripts fail early.
        build_script_contents += s_fail.format(build_name=build_name,
                                               fail_stmt=fail_stmt,
                                               **kwargs)

    build_script_file = "build_" + build_name + script_ext
    tools.write_to_file(build_script_file, build_script_contents,
                        force_unix=False)
    return build_script_file


def _run_script(script):
    if sys.platform in ("win32", "cygwin"):
        shell = ["cmd", "/c"]
    else:
        shell = ["bash"]

    if subprocess.call(shell + [script]) != 0:
        raise OSError("Subprocess failed")


class LatticeIceStormToolchain:
    attr_translate = {
        "keep": ("keep", "true"),
        "no_retiming": None,  # yosys can do retiming via the (non-default)
                              # "-retime" option to "synth_ice40", but
                              # yosys does not check for an equivalent
                              # constraint to prevent retiming on signals.
        "async_reg": None,  # yosys has no equivalent, and arachne-pnr
                            # wouldn't take advantage of it anyway.

        # While custom attributes are supported in yosys, neither
        # arachne-pnr nor icetime currently can take advantage of them
        # to add fine-grained timing constraints.
        "mr_ff": None,  # user-defined attribute
        "mr_false_path": None,  # user-defined attribute
        "ars_ff1": None,  # user-defined attribute
        "ars_ff2": None,  # user-defined attribute
        "ars_false_path": None,  # user-defined attribute

        # ice40 does not have a shift register primitive.
        "no_shreg_extract": None
    }

    special_overrides = common.lattice_ice40_special_overrides

    def __init__(self):
        # Variables within replacement fields should be backend-aware and
        # update their syntax accordingly. Currently, only {pnr_pkg_opts}
        # needs this functionality.

        self.yosys_template = [
            "{read_files}",
            "attrmap -tocase keep -imap keep=\"true\" keep=1 -imap keep=\"false\" keep=0 -remove keep=0",
            "synth_ice40 -top {build_name} -blif {build_name}.blif",
        ]

        self.build_template = [
            "yosys -q -l {build_name}.rpt {build_name}.ys",
            "arachne-pnr -q -l {pnr_pkg_opts} -p {build_name}.pcf {build_name}.blif -o {build_name}.txt",
            "icetime {icetime_pkg_opts} -c {freq_constraint} -t -p {build_name}.pcf -r {build_name}.tim {build_name}.txt",
            "icepack {build_name}.txt {build_name}.bin"
        ]

        self.nextpnr_yosys_template = [
            "{read_files}",
            "attrmap -tocase keep -imap keep=\"true\" keep=1 -imap keep=\"false\" keep=0 -remove keep=0",
            "synth_ice40 {synth_opts} -top {build_name} -json {build_name}.json",
        ]

        self.nextpnr_build_template = [
            "yosys -q -l {build_name}.rpt {build_name}.ys",
            "nextpnr-ice40 {pnr_pkg_opts} --pcf {build_name}.pcf --json {build_name}.json --asc {build_name}.txt --pre-pack {build_name}_pre_pack.py",
            "icepack {build_name}.txt {build_name}.bin"
        ]

        self.freq_constraints = dict()

    # platform.device should be of the form "ice40-{lp384, hx1k, etc}-{tq144, etc}"
    def build(self, platform, fragment, build_dir="build", build_name="top",
              use_nextpnr=True, synth_opts="", run=True, **kwargs):
        os.makedirs(build_dir, exist_ok=True)
        cwd = os.getcwd()
        os.chdir(build_dir)

        if not isinstance(fragment, _Fragment):
            fragment = fragment.get_fragment()
        platform.finalize(fragment)

        v_output = platform.get_verilog(fragment, name=build_name, **kwargs)
        named_sc, named_pc = platform.resolve_signals(v_output.ns)
        v_file = build_name + ".v"
        v_output.write(v_file)

        if use_nextpnr:
            chosen_yosys_template = self.nextpnr_yosys_template
        else:
            chosen_yosys_template = self.yosys_template
        ys_contents = "\n".join(_.format(build_name=build_name,
                                         read_files=self.gen_read_files(platform, v_file),
                                         synth_opts=synth_opts)
                                for _ in chosen_yosys_template)

        ys_name = build_name + ".ys"
        tools.write_to_file(ys_name, ys_contents)

        tools.write_to_file(build_name + ".pcf",
                            _build_pcf(named_sc, named_pc))
        (family, series_size, package) = self.parse_device_string(platform.device)
        if use_nextpnr:
            pnr_pkg_opts = "--" + series_size + " --package " + package
        else:
            pnr_pkg_opts = "-d " + self.get_size_string(series_size) + \
                           " -P " + package
        icetime_pkg_opts = "-P " + package + " -d " + series_size

        if use_nextpnr:
            tools.write_to_file(build_name + "_pre_pack.py",
                                _build_pre_pack(v_output.ns, self.freq_constraints))
        # icetime can only handle a single global constraint, so we test against the fastest
        # clock; though imprecise, if the global design satisfies the fastest clock, we can
        # be sure all other constraints are satisfied.
        freq_constraint = str(max(self.freq_constraints.values(),
                                  default=0.0))

        if use_nextpnr:
            chosen_build_template = self.nextpnr_build_template
        else:
            chosen_build_template = self.build_template
        script = _build_script(source=False,
                               build_template=chosen_build_template,
                               build_name=build_name,
                               pnr_pkg_opts=pnr_pkg_opts,
                               icetime_pkg_opts=icetime_pkg_opts,
                               freq_constraint=freq_constraint)

        if run:
            _run_script(script)

        os.chdir(cwd)

        return v_output.ns

    def parse_device_string(self, device_str):
        # Arachne only understands packages based on the device size, but
        # LP for a given size supports packages that HX for the same size
        # doesn't and vice versa; we need to know the device series due to
        # icetime.
        valid_packages = {
            "lp384": ["qn32", "cm36", "cm49"],
            "lp1k": ["swg16tr", "cm36", "cm49", "cm81", "cb81", "qn84",
                     "cm121", "cb121"],
            "hx1k": ["vq100", "cb132", "tq144"],
            "lp8k": ["cm81", "cm81:4k", "cm121", "cm121:4k", "cm225",
                     "cm225:4k"],
            "hx8k": ["bg121", "bg121:4k", "cb132", "cb132:4k", "cm121",
                     "cm121:4k", "cm225", "cm225:4k", "cm81", "cm81:4k",
                     "ct256", "tq144:4k"],
            "up3k": ["sg48", "uwg30"],
            "up5k": ["sg48", "uwg30"],
        }

        (family, series_size, package) = device_str.split("-")
        if family not in ["ice40"]:
            raise ValueError("Unknown device family")
        if series_size not in ["lp384", "lp1k", "hx1k", "lp8k", "hx8k", "up5k"]:
            raise ValueError("Invalid device series/size")
        if package not in valid_packages[series_size]:
            raise ValueError("Invalid device package")
        return (family, series_size, package)

    def get_size_string(self, series_size_str):
        return series_size_str[2:]

    def gen_read_files(self, platform, main):
        sources = platform.sources | {(main, "verilog", "work")}
        incflags = ""
        read_files = list()
        for path in platform.verilog_include_paths:
            incflags += " -I" + path
        for filename, language, library in sources:
            read_files.append("read_{}{} {}".format(language,
                                                    incflags,
                                                    filename))
        return "\n".join(read_files)

    def add_period_constraint(self, platform, clk, period):
        new_freq = 1000.0/period

        if clk not in self.freq_constraints.keys():
            self.freq_constraints[clk] = new_freq
        else:
            raise ConstraintError("Period constraint already added to signal.")