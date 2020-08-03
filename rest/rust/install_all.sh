curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
. ~/.cargo/env
rustup default nightly

echo source ~/.cargo/env >> ~/.bashrc

