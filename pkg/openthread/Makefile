PKG_NAME=openthread
PKG_URL=git://github.com/openthread/openthread.git
PKG_VERSION=f6ff6fcd85a1661c14b314995d4b9ebae1600fd9
PKG_BUILDDIR ?= $(RIOTBASE)/pkg/$(PKG_NAME)/source



.PHONY: all

all: git-download
		$(MAKE) -C $(PKG_BUILDDIR)

include $(RIOTBASE)/pkg/pkg.mk
