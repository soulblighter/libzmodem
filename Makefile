CC ?= gcc
CXX?= g++
AR ?= ar
RM := rm -rf
MD := mkdir -p
CP := cp


WFLAGS := -Wall -Wextra -Wchar-subscripts -Wdouble-promotion -Wformat \
        -Wfloat-equal -Wshadow -Wundef -Wcast-qual -Wwrite-strings \
        -Wconversion -Wlogical-op -Wredundant-decls -fdiagnostics-color=always


CFLAGS += $(WFLAGS) -fno-inline -fPIC -fPIE
CFLAGS += -Isrc -Iinclude


INC_DIRS                  += $(shell find src -type d -name "*")
CFLAGS                    += $(addprefix -I, $(INC_DIRS))


OUT_DIR = .build/$(shell $(CC) -dumpmachine | cut -d- -f1)


LIBS                      += -lm
LFLAGS                    += $(LIBS)


C_FILES                   := $(shell find src -type f -name "*.c" | grep -v "mrz\|msz")
OBJS                      := $(addprefix $(OUT_DIR)/, $(C_FILES))
OBJS                      := $(OBJS:.c=.o)
DEPS                      := $(OBJS:.o=.d)



.PHONY: clean all
.NOTPARALLEL: log

all: log libzmodem mrz msz


mrz: $(OUT_DIR)/mrz

msz: $(OUT_DIR)/msz

libzmodem: $(OUT_DIR)/libzmodem.a


$(OUT_DIR)/libzmodem.a: $(OBJS)
	@echo "AR $@"
	@$(AR) -rc $@ $^

$(OUT_DIR)/mrz: $(OUT_DIR)/src/mrz.o
	@echo "CC $@"
	@$(CC) $(LDFLAGS) -L$(OUT_DIR) $^ $(LFLAGS) -lzmodem -o $@

$(OUT_DIR)/msz: $(OUT_DIR)/src/msz.o
	@echo "CC $@"
	@$(CC) $(LDFLAGS) -L$(OUT_DIR) $^ $(LFLAGS) -lzmodem -o $@

$(OUT_DIR)/%.o : %.c
	@mkdir -p $(dir $@)
	@echo "CC $<"
	@$(CC) -c $(CFLAGS) $< -o $@

clean:
	$(RM) $(OUT_DIR)

log:
	@printf "\nBuilding $(LOCAL_MODULE)...\n"
	@printf "CC       : $(CC)\n"
	@printf "CXX      : $(CXX)\n"
	@printf "CFLAGS   : $(CFLAGS)\n"
	@printf "LFLAGS   : $(LFLAGS)\n"
	@printf "\n\n"
