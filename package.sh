srcdir=`pwd`
bindir=hbpintool-release
tarball=${bindir}.tar.gz

rm -rf ${bindir} ${tarball}

mkdir -p ${bindir}
cp -r ${srcdir}/configure   ${bindir}
cp -r ${srcdir}/obj-intel64 ${bindir}
cp -r ${srcdir}/SOURCE_THIS ${bindir}
cp -r ${srcdir}/hbpintool   ${bindir}
cp -r ${srcdir}/README.md   ${bindir}

tar czf ${tarball} ${bindir}
