PKG_NAME=openthread
PKG_URL=http://github.com/Hyungsin/openthread.git
PKG_VERSION=4d8304c234d1565a1e4d0ec37ad2e04efd498421
PKG_BUILDDIR ?= $(RIOTBASE)/pkg/$(PKG_NAME)/source

.PHONY: all

all: git-download
		$(MAKE) -C $(PKG_BUILDDIR)

include $(RIOTBASE)/pkg/pkg.mk
