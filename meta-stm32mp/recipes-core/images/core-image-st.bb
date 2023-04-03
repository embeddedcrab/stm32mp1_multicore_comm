SUMMARY = "STM32MP1 Custom image"

IMAGE_INSTALL = "packagegroup-core-boot ${CORE_IMAGE_EXTRA_INSTALL}"

IMAGE_LINGUAS = " "

LICENSE = "MIT"

inherit core-image
inherit extrausers


IMAGE_OVERHEAD_FACTOR ?= "1.0"
IMAGE_ROOTFS_SIZE ?= "8192"

INHERIT += "extrausers"
EXTRA_USERS_PARAMS += "usermod -p root root;"

IMAGE_ROOTFS_EXTRA_SPACE_append = "${@bb.utils.contains("DISTRO_FEATURES", "systemd", " + 4096", "" ,d)}"

# Install applications in custom build
IMAGE_INSTALL +=	\
	"hello-world	\
	rpcomm		\
	ht-server	\
	ht-client"

