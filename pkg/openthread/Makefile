PKG_NAME=openthread
PKG_URL=http://github.com/Hyungsin/openthread.git
# PKG_VERSION can be a branch or commit
PKG_VERSION=basic_compile
PKG_BUILDDIR ?= $(RIOTBASE)/pkg/$(PKG_NAME)/source

.PHONY: all

all: git-download
		$(MAKE) -C $(PKG_BUILDDIR)

include $(RIOTBASE)/pkg/pkg.mk
