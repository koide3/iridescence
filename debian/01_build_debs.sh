#!/bin/bash
VERSION=1.0.2
MESSAGE="backface culling"

export DEBFULLNAME="Kenji Koide"
export DEBEMAIL="k.koide@aist.go.jp"

pushd .

cd ..

dch -v $VERSION-1ppa1~jammy1 "$MESSAGE" -D jammy
git add debian/changelog
git commit -m "update debian/changelog for jammy"
gbp buildpackage -S -sa --git-upstream-tag='v%(version)s' --git-submodules

dch -v $VERSION-1ppa1~noble1 "$MESSAGE" -D noble
git add debian/changelog
git commit -m "update debian/changelog for noble"
gbp buildpackage -S -sa --git-upstream-tag='v%(version)s' --git-submodules

dch -v $VERSION-1ppa1~resolute1 "$MESSAGE" -D resolute
git add debian/changelog
git commit -m "update debian/changelog for resolute"
gbp buildpackage -S -sa --git-upstream-tag='v%(version)s' --git-submodules

popd

echo "Please run 'dput ppa:koide3/iridescence ../*.changes' to upload the package to the PPA."
