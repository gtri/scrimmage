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

# References:
# https://gist.github.com/domenic/ec8b0fc8ab45f39403dd
# https://github.com/alrra/travis-scripts/blob/master/doc/github-deploy-keys.md

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

# Create a tarball of the documentation
tar -czf doc_output.tar.gz ./doc_output

# Send the tarball to the document server
scp ./doc_output.tar.gz kevindem@scrimmagesim.org:~/public_html/scrimmagesim

# ssh into the documentation server.
# 1. Remove the old docs folder.
# 2. Recreate the docs folder.
# 3. Untar the docs
# 4. Move the sphinx and doxygen html output to the correct directories.
ssh kevindem@scrimmagesim.org 'cd public_html/scrimmagesim; rm -rf ./docs; mkdir -p ./docs; tar xf doc_output.tar.gz; mv ./doc_output/sphinx ./docs/; mv ./doc_output/doxygen ./docs/'
