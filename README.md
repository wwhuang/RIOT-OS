# Hamilton-combined-v9.0 - July 5th 2017

Board support for the Hamilton mote is maintained as a set of rebasing branches
that will at some stage be pushed upstream. At intervals, a "combined" branch
is created so that working with the hamilton is as easy as cloning this repo.
The v9.0 branch was rebased with "RIOT release 2017.07" and created with the following commands. 

```bash
git clone https://github.com/Hyungsin/RIOT-OS.git
cd RIOT-OS
git remote add upstream https://github.com/RIOT-OS/RIOT.git
git checkout origin/master 
# this was 
commit 219ffb38479710ade70767493fb824f93e8b9c14
Merge: e91c077 efcc275
Author: Thomas Eichinger <thomas.eichinger1@gmail.com>
Date:   Tue Jul 4 13:48:24 2017 -0700

    Merge pull request #7283 from smlng/dist/tools/edbg/fix_macos
    
    tools, edbg: fix compiler issue on 

git checkout -b hamilton-combined-v9.0
git fetch upstream
git pull upstream/master

# From upstream PR (miscellaneous fixes not merged yet)
git fetch upstream pull/5969/head:pr-5969
git merge --no-ff pr-5969 #at30ts74

git fetch upstream pull/5970/head:pr-5970
git merge --no-ff pr-5970 #mma7660

git fetch upstream pull/7307/head:pr-7307
git merge --no-ff pr-7307 #gpio fix

git fetch upstream pull/7308/head:pr-7308
git merge --no-ff pr-7308 #pm configuration fix

git fetch upstream pull/7309/head:pr-7309
git merge --no-ff pr-7309 #timer independent radio state change

# Hamilton CPU and Board
git merge --no-ff origin/hamilton-board
git merge --no-ff origin/hamilton-clock 
# We are using our own adc implementation
git merge --no-ff origin/hamilton-adc   

# Hamilton dutycycling MAC (listen-after-send)
git merge --no-ff origin/hamilton-lasmac

# sensor drivers (SAUL-compatible)
git merge --no-ff origin/hamilton-pushbutton
git merge --no-ff origin/hamilton-fxos8700 
git merge --no-ff origin/hamilton-ekmb1101111
git merge --no-ff origin/hamilton-apds9007 
git merge --no-ff origin/hamilton-tmp006

# then this readme was edited
git commit -m "icing: add readme"
git push --set-upstream origin hamilton-combined-v9.0
```

# Average Power Consumption

The power consumption of a hamilton-7C (without the PIR sensor) with this firmware has been measured at

(1) When using the 48 MHz main clock (DFLL)

 INTERVAL  | POWER CONSUMPTION | IDEAL BATTERY LIFE  |
 --------- | ----------------- | ------------------- |
  10 s     | 42 uA             | 4.0 years           |
  20 s     | 25 uA             | 6.8 years           |
  30 s     | 19 uA             | 9.0 years           |
  NEVER    | 6.2 uA            | 27.6 years          |

(2) When using the 8 MHz main clock (OSC8M)

 INTERVAL  | POWER CONSUMPTION | IDEAL BATTERY LIFE  |
 --------- | ----------------- | ------------------- |
  10 s     | 34 uA             | 5.0 years           |
  20 s     | 21 uA             | 8.1 years           |
  30 s     | 16 uA             | 10.7 years          |
  NEVER    | 6.2 uA            | 27.6 years          |

The ideal battery life is assuming a 1500mAh battery with no self-discharge. In
real life, results will vary. The power consumption is given as the current term, multiply by the system voltage to get the power. We have found that the whole system current only varies slightly with voltage, so it is more useful to record the current than the power. (In other words at 2.2v instead of 3.3v the idle current will still be 6.2 uA but the "power" will drop by 33%).

# Further work list for Hamilton-combined-v9.x
1) Xtimer improvement: HYUNG
2) OpenThread test: HYUNG
3) Software CSMA: SAM
4) TCPlp: SAM
5) LASMAC fix to support TCPlp: SAM/HYUNG
6) Sensor driver optimization: MICHAEL/HYUNG

If you want to contribute, please consider contributing upstream. If that is
not appropriate (you have hamilton-specific changes) please submit a PR
as changes on top of master (which will track upstream) as this makes rebasing
easier. If that is not possible (you are editing hamilton-specific files) you
can base your PR on a combined branch, but please make clear which version
you used. We recommend including this information in your branches, such as
`c9.0-my-feature`.

# Upstream readme

                          ZZZZZZ
                        ZZZZZZZZZZZZ
                      ZZZZZZZZZZZZZZZZ
                     ZZZZZZZ     ZZZZZZ
                    ZZZZZZ        ZZZZZ
                    ZZZZZ          ZZZZ
                    ZZZZ           ZZZZZ
                    ZZZZ           ZZZZ
                    ZZZZ          ZZZZZ
                    ZZZZ        ZZZZZZ
                    ZZZZ     ZZZZZZZZ       777        7777       7777777777
              ZZ    ZZZZ   ZZZZZZZZ         777      77777777    77777777777
          ZZZZZZZ   ZZZZ  ZZZZZZZ           777     7777  7777       777
        ZZZZZZZZZ   ZZZZ    Z               777     777    777       777
       ZZZZZZ       ZZZZ                    777     777    777       777
      ZZZZZ         ZZZZ                    777     777    777       777
     ZZZZZ          ZZZZZ    ZZZZ           777     777    777       777
     ZZZZ           ZZZZZ    ZZZZZ          777     777    777       777
     ZZZZ           ZZZZZ     ZZZZZ         777     777    777       777
     ZZZZ           ZZZZ       ZZZZZ        777     777    777       777
     ZZZZZ         ZZZZZ        ZZZZZ       777     777    777       777
      ZZZZZZ     ZZZZZZ          ZZZZZ      777     7777777777       777
       ZZZZZZZZZZZZZZZ            ZZZZ      777      77777777        777
         ZZZZZZZZZZZ               Z
            ZZZZZ

The friendly Operating System for IoT!

RIOT is a real-time multi-threading operating system that supports a range of
devices that are typically found in the Internet of Things (IoT): 
8-bit, 16-bit and 32-bit microcontrollers.

RIOT is based on the following design principles: energy-efficiency, real-time
capabilities, small memory footprint, modularity, and uniform API access,
independent of the underlying hardware (this API offers partial POSIX
compliance).

RIOT is developed by an international open source community which is
independent of specific vendors (e.g. similarly to the Linux community).
RIOT is licensed with LGPLv2.1, a copyleft license which fosters
indirect business models around the free open-source software platform
provided by RIOT, e.g. it is possible to link closed-source code with the
LGPL code.

## FEATURES

RIOT is based on a microkernel architecture, and provides features including,
but not limited to:

* a preemptive, tickless scheduler with priorities
* flexible memory management
* high resolution, long-term timers
* support for AVR, MSP430, MIPS, ARM7, and ARM Cortex-M on over 80 boards
* the native port allows to run RIOT as-is on Linux, BSD, and MacOS. Multiple
  instances of RIOT running on a single machine can also be interconnected via
  a simple virtual Ethernet bridge
* IPv6
* 6LoWPAN (RFC4944, RFC6282, and RFC6775)
* UDP
* RPL (storing mode, P2P mode)
* CoAP
* CCN-Lite


## GETTING STARTED
* You want to start the RIOT? Just follow our [quickstart guide](http://doc.riot-os.org/index.html#the-quickest-start) or the [getting started documentation](http://doc.riot-os.org/getting-started.html).
* The RIOT API itself can be built from the code using doxygen. The latest
  version is uploaded daily to http://riot-os.org/api.

## KNOWN ISSUES
* With latest GCC version (>= 6) platforms based on some ARM platforms will
  raise some warnings, leading to a failing build
  (see https://github.com/RIOT-OS/RIOT/issues/5519).
  As a workaround, you can compile with warnings not being treated as errors:
  `WERROR=0 make`

### USING THE NATIVE PORT WITH NETWORKING
If you compile RIOT for the native cpu and include the `netdev_tap` module,
you can specify a network interface like this: `PORT=tap0 make term`

#### SETTING UP A TAP NETWORK
There is a shellscript in `RIOT/dist/tools/tapsetup` called `tapsetup` which
you can use to create a network of tap interfaces.

*USAGE*
To create a bridge and two (or count at your option) tap interfaces:

    ./dist/tools/tapsetup/tapsetup [-c [<count>]]

## CONTRIBUTE

To contribute something to RIOT, please refer to the [development
procedures](https://github.com/RIOT-OS/RIOT/wiki/Development-procedures) and
read all notes for best practice.

## MAILING LISTS
* RIOT OS kernel developers list
 * devel@riot-os.org (http://lists.riot-os.org/mailman/listinfo/devel)
* RIOT OS users list
 * users@riot-os.org (http://lists.riot-os.org/mailman/listinfo/users)
* RIOT commits
 * commits@riot-os.org (http://lists.riot-os.org/mailman/listinfo/commits)
* Github notifications
 * notifications@riot-os.org
   (http://lists.riot-os.org/mailman/listinfo/notifications)

## LICENSE
* Most of the code developed by the RIOT community is licensed under the GNU
  Lesser General Public License (LGPL) version 2.1 as published by the Free
  Software Foundation.
* Some external sources, especially files developed by SICS are published under
  a separate license.

All code files contain licensing information.

For more information, see the RIOT website:

http://www.riot-os.org
