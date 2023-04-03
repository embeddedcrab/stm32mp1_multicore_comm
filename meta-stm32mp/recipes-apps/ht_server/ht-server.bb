#
# Remote Processor Server
#

DESCRIPTION = "RP-Communication Server Application for Yocto build."
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta/COPYING.MIT;md5=3da9cfbcb788c80a0384361b4de20420"


inherit externalsrc pkgconfig cmake


# Absolute Path of src CMakeLists.txt
EXTERNALSRC = "/home/hemant/stm32mp157/stm-apps/mp157/HtServer"

DEPENDS += "utils"

EXTRA_OECMAKE = ""


# The autotools configuration I am basing this on seems to have a problem with a race condition when parallel make is enabled
PARALLEL_MAKE = ""

