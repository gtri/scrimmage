#!/bin/bash
set -e # Exit with nonzero exit code if anything fails

# Ensure documentation folders exist
if [ ! -d "./doc_output/sphinx" ]; then
    echo "ERROR: sphinx documentation doesn't exist."
    exit 1
fi
if [ ! -d "./doc_output/doxygen" ]; then
    echo "ERROR: doxygen documentation doesn't exist."
    exit 1
fi
if [ ! -d "./doc_output/jekyll" ]; then
    echo "ERROR: jekyll documentation doesn't exist."
    exit 1
fi

SHA=`git rev-parse --verify HEAD`

# Get the deploy key by using Travis's stored variables to decrypt deploy_key.enc
ENCRYPTED_KEY_VAR="encrypted_${ENCRYPTION_LABEL}_key"
ENCRYPTED_IV_VAR="encrypted_${ENCRYPTION_LABEL}_iv"
ENCRYPTED_KEY=${!ENCRYPTED_KEY_VAR}
ENCRYPTED_IV=${!ENCRYPTED_IV_VAR}
openssl aes-256-cbc -K $ENCRYPTED_KEY -iv $ENCRYPTED_IV -in ./ci/keys/deploy_key.enc -out ./ci/keys/deploy_key -d

# Setup ssh-agent to use decrypted deployment key
chmod 600 ./ci/keys/deploy_key
eval `ssh-agent -s`
ssh-add ./ci/keys/deploy_key

git clone --single-branch --branch gh-pages git@github.com:gtri/scrimmage.git gh-pages-dir
pushd gh-pages-dir
    git config user.name "Travis CI"
    git config user.email "deploy@travis.ci"
    cp -r ../doc_output/jekyll/* .
    cp -r ../doc_output/sphinx .
    cp -r ../doc_output/doxygen .

    # If there are no changes to the compiled out, don't push
    if git diff --quiet; then
        echo "No changes to the output on this push; exiting."
        exit 0
    fi

    git add -A .
    git commit -m "Deploy to GitHub Pages: ${SHA}"
    git push origin gh-pages
popd
