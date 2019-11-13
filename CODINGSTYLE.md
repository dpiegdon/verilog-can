# vim: colorcolumn=80 fo+=a

General verilog coding style
============================

Overall the style is similar to the C coding style of the linux kernel, adapted
to verilog.


* Code- and textwidth should not be more than 80 characters.

* Indent using a single tab (and not space(s)). A single tab may be assumed to
  be 8 spaces wide, but it is and stays a tab.

* Remove needless whitespace, e.g. at end of lines.


* Always place `begin` and `end`, even if optional.

* Place `begin` on same line as corresponding statement (e.g. with `for`,
  `else`, `if` or `always`).

* Always break the line after `begin`. Exception: place `begin` and `end`
  separately for `fork` children.

* If a long statement with a `begin` at the end has to be broken up (e.g. long
  if-statement), leave an empty line before any further code.

* `end` may be followed by `else begin` or `else if(...) begin`.

* No other code should follow an `end`.


* `if` and `for` are followed by a brace, without space. `if(a=0) begin`

* `always @(...)`: No space between `@` and brace.


* Avoid macros and defines where possible. Use parameters or refactor your code
  such that you gain a backend with multiple frontend implementations that
  introduce the required changes.

* For a larger module `mymodule`, have a single file `mymodule.v`

* Smaller modules and helper-modules may be collected in a single file.

* Always set the default nettype to none.

* Module inputs and outputs must be fully defined and qualified in the
  `module()` statement: `module mymod(input wire xyz, output reg [7...0] foo)`.
  Do not insert `input xyz` or `wire xyz` statements inside the module
  definition.

* Module inputs and outputs should not be marked with suffixes like `_i` or
  `_o`.

* Use sensible i/o and variable names that can be understood by seeing them.
  Use `bit_ready` instead of `rdy` or `shiftreg` instead of `s`.

* Full module statement including i/o definitions should not be more than a few
  lines, preferably single line. Consider refactoring if that is not the case.
  Only if that is (currently) not possible the i/o variables should be listed
  one per line.

* Variable names are preferred to be lower case with underscore. Same goes for
  module names.


* Track your code in a git repo (git commit style is not part of this
  document).


Testbenches and simulation
==========================

* Each module should have a testbench that does at least some minimal i/o
  tests.

* Testbench for module `mymod` should be named `mymod_tb` and be in file
  `mymod_tb.v`

* For a proper signal delay simulation, read the below document. LHS and RHS
  delay operator are usually not the correct way.
  http://www-inst.eecs.berkeley.edu/~cs152/fa06/handouts/CummingsHDLCON1999_BehavioralDelays_Rev1_1.pdf

* Testbench errors should be added up. At the end, if there were more than zero
  errors, the result should be `$fatal()`. Otherwise `$finish`

* Provide a make-target to run all known testbenches (*_tb.v). Run it before
  every commit to see if your code still works properly.

