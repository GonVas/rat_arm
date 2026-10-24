{
  description = "ESP32-C3 Rust Development Environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    rust-overlay.url = "github:oxalica/rust-overlay";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, rust-overlay, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        overlays = [ (import rust-overlay) ];
        pkgs = import nixpkgs { inherit system overlays; };
        
        # Define the exact Rust toolchain with the RISC-V target
        rustToolchain = pkgs.rust-bin.stable.latest.default.override {
          targets = [ "riscv32imc-unknown-none-elf" ];
          extensions = [ "rust-src" "rust-analyzer" ];
        };
      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = with pkgs; [
            rustToolchain
            
            # Tools for flashing and code generation
            espflash
            esp-generate
            
            # Native requirements for building firmware & crypto dependencies
            pkg-config
            openssl
            udev
            blender
            orca-slicer
          ];

          # Set up environment variables so build scripts find everything seamlessly
          shellHook = ''
            export PKG_CONFIG_PATH="${pkgs.openssl.dev}/lib/pkg-config"
          '';
        };
      }
    );
}
