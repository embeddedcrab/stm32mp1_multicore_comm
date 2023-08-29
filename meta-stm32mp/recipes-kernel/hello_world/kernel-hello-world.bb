#
# Kernel Hello World
#

SUMMARY = "Extern Hello World Linux Kernel Module"
DESCRIPTION = "Hello World Kernel Sample Driver"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta/files/common-licenses/GPL-2.0;md5=801f80980d171dd6425610833a22dbe6"

inherit module externalsrc

#FILESEXTRAPATHS_prepend = 

SRC_URI += " \
    file://Makefile \
    file://hello_world.c \
    "

S = "${WORKDIR}"

DEPENDS += "virtual/kernel"

MODULES_INSTALL_TARGET="install"

do_compile () {
    oe_runmake
}

do_install () {
    install -d ${D}/lib/modules/${KERNEL_VERSION}/
    install -m 0755 ${B}/hello_world.ko ${D}/lib/modules/${KERNEL_VERSION}/
}

FILES_${PN} += "${nonarch_base_libdir}/modules/${KERNEL_VERSION}/hello_world.ko"
RPROVIDES_${PN} += "kernel-module-${PN}-${KERNEL_VERSION}"