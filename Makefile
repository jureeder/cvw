#make all: submodules other
#make all: submodules other
#submodules: addins/riscv-isa-sim addins/riscv-arch-test
#	git pull --recurse-submodules
#
#other:
make all:
# move these parts into compiling archtest separtately
	cp -r addins/riscv-isa-sim/arch_test_target/spike/device/rv32i_m/I addins/riscv-isa-sim/arch_test_target/spike/device/rv32i_m/F
	cp -r addins/riscv-isa-sim/arch_test_target/spike/device/rv64i_m/I addins/riscv-isa-sim/arch_test_target/spike/device/rv64i_m/D
<<<<<<< HEAD
#why cat
	cat addins/riscv-isa-sim/arch_test_target/spike/device/rv32i_m/F/Makefile.include
=======
>>>>>>> 29f2a1c5479d7a80debdb1ac337fcda628cc57a3
	sed -i 's/--isa=rv32i /--isa=32if/' addins/riscv-isa-sim/arch_test_target/spike/device/rv32i_m/F/Makefile.include
	sed -i 's/--isa=rv64i /--isa=64if/' addins/riscv-isa-sim/arch_test_target/spike/device/rv64i_m/D/Makefile.include
	if [ -d "addins/riscv-isa-sim/build" ]; then echo "Build exists"; else mkdir addins/riscv-isa-sim/build; fi
	cd addins/riscv-isa-sim/build; ../configure --prefix=/cad/riscv/gcc/bin
	make -C addins/riscv-isa-sim/build
# does sudo work?

	sudo make install -C addins/riscv-isa-sim/build

	cp addins/riscv-isa-sim/arch_test_target/spike/Makefile.include addins/riscv-arch-test/
# update with path including $RISCV_TOOLS
# separate into make tests and make regression
	sed -i '/export TARGETDIR ?=/c\export TARGETDIR ?= /home/harris/riscv-wally/addins/riscv-isa-sim/arch_test_target' tests/wally-riscv-arch-test/Makefile.include
	echo export RISCV_PREFIX = riscv64-unknown-elf- >> tests/wally-riscv-arch-test/Makefile.include
	make -C addins/riscv-arch-test
	make -C addins/riscv-arch-test XLEN=32
	cd tests/wally-riscv-arch-test; exe2memfile.pl work/*/*/*.elf
	make -C wally-pipelined/regression
	


