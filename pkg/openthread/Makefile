PKG_NAME=openthread
PKG_URL=git://github.com/openthread/openthread.git
PKG_VERSION=6d48a7b34da581cc329af4f35357745a416b3981
PKG_BUILDDIR ?= $(RIOTBASE)/pkg/$(PKG_NAME)/source



.PHONY: all

all: git-download
		$(MAKE) -C $(PKG_BUILDDIR)

include $(RIOTBASE)/pkg/pkg.mk
