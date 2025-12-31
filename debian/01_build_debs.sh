#!/bin/bash
export DEBFULLNAME="Kenji Koide"
export DEBEMAIL="k.koide@aist.go.jp"

pushd .

cd ..

dch -v $1 $2 -D noble
git add debian/changelog
git commit -m "update debian/changelog for noble"
gbp buildpackage -S -sa --git-upstream-tag='v%(version)s' --git-submodules

dch -v $1 $2 -D jammy
git add debian/changelog
git commit -m "update debian/changelog for jammy"
gbp buildpackage -S -sa --git-upstream-tag='v%(version)s' --git-submodules

popd