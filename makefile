
all: release

release:
	mkdir -p build/release
	cd build/release && cmake -DCMAKE_BUILD_TYPE=Release ../.. && make -j`nproc`

prerelease:
	mkdir -p build/prerelease
	cd build/prerelease && cmake -DCMAKE_BUILD_TYPE=Prerelease ../.. && make -j`nproc`

debug:
	mkdir -p build/debug
	cd build/debug && cmake -DCMAKE_BUILD_TYPE=Debug ../.. && make -j`nproc`

clean:
	rm -rf build

install: release
	cd build/release && sudo make install

####################################################################
LINUX_DEPLOY_QT = linuxdeployqt-continuous-x86_64.AppImage
qt_apps: qmake make_qt

qmake:
	qmake RTKLib.pro -spec linux-g++ -o QtMakefile

make_qt: qmake
	make -f QtMakefile -j `nproc`

clean_qt:
	make -f QtMakefile clean -j `nproc`
	rm QtMakefile
	rm -rf build/Qt

qt_appimages: qt_apps
	./util/build_scripts/AppImageDeploy.sh $(LINUX_DEPLOY_QT)

qt_debs: qt_apps
	./util/build_scripts/debdeploy.sh