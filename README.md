<div id="top"></div>

<!-- Project Shields -->

<p align="center">

  [![Stargazers][stars-shield]][stars-url]
  [![Forks][forks-shield]][forks-url]
  [![MIT License][license-shield]][license-url]

</p>

<!-- PROJECT LOGO -->

<br />
<div align="center">
<!--
  <a href="https://github.com/github_username/repo_name">
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a>
-->
  <h3 align="center">Rusty Feather</h3>
<!--
  <p align="center">
    project_description
    <br />
    <a href="https://github.com/github_username/repo_name"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/github_username/repo_name">View Demo</a>
    ·
    <a href="https://github.com/github_username/repo_name/issues">Report Bug</a>
    ·
    <a href="https://github.com/github_username/repo_name/issues">Request Feature</a>
  </p>
-->
</div>

## About the Project

Rusty Feather is an experiment with Rust on the [Adafruit RP2040](https://www.adafruit.com/product/4884) Feather and several Feather and StemmaQT boards.

## Getting Started

Make sure you have the latest stable version of Rust installed, along with the right target support.

```
rustup self update
rustup update stable
rustup target add thumbv6m-none-eabi
```

Other stuff?

```
#cargo install cargo-hf2
cargo install elf2uf2-rs --locked
cargo install cargo-binutils
```

Next, clone the project.

```
git clone https://github.com/jstevej/rusty-feather.git
```

Build, check size, convert from elf to hf2, load on RP2040.

```
cargo build --release
cargo size --release
cargo size --release --bin rusty-feather -- -A
elf2uf2-rs target/thumbv6m-none-eabi/release/rusty-feather rusty-feather.uf2
cp rusty-feather.uf2 /Volumes/RPI-RP2
```

There are probably a lot of options out there for accessing the USB serially, but the simplest is `screen`. First, take a look at `ls /dev/tty.*` and figure out which one is the RP2040. Then, run `screen /dev/tty.yourRP2040`. Use Ctrl-A Ctrl-D to exit `screen`.

Useful tool: `cargo tree` and `cargo tree -d` (show duplicates).

How to update `Cargo.toml` dependencies?

If starting from scratch note the following:

* add `.cargo/config` from [here](https://github.com/rp-rs/rp-hal/blob/main/.cargo/config)
* add `memory.x` to project root from [here](https://github.com/rp-rs/rp-hal/blob/main/memory.x)

<p align="right">(<a href="#top">back to top</a>)</p>

## Roadmap

  - [x] Red LED
  - [x] Neopixel
  - [ ] USB serial
    - [x] send and receive with interrupts
    - [ ] producer and consumer queues
  - [ ] CO2 sensor
  - [ ] E-Ink display
  - [ ] VOC sensor
  - [ ] Magnetometers
  - [ ] Accelerometers
  - [ ] make runner to automate `elf2uf2-rs` and `cp` steps, and/or...
  - [ ] figure out why `cargo hf2 --release` doesn't work
  - [ ] Buttons
  - [ ] Knobs
  - [ ] Servo
  - [ ] SH1107 Display
    - [SH1107 Display Driver](https://github.com/aaron-hardin/sh1107)

<p align="right">(<a href="#top">back to top</a>)</p>

<!--
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#top">back to top</a>)</p>
-->

## License

Distributed under the MIT License. See `LICENSE` for more information.

<p align="right">(<a href="#top">back to top</a>)</p>

<!--
## Contact

Your Name - [@twitter_handle](https://twitter.com/twitter_handle) - email@email_client.com

Project Link: [https://github.com/github_username/repo_name](https://github.com/github_username/repo_name)

<p align="right">(<a href="#top">back to top</a>)</p>
-->


## Acknowledgments and Resources

* [The Rust Programming Language](https://doc.rust-lang.org/book/)
* [The Embedded Rust Book](https://docs.rust-embedded.org/book/)
* [Embedded HAL](https://docs.rs/embedded-hal/latest/embedded_hal/)
* [RP2040 HAL](https://github.com/rp-rs/rp-hal)
* [WS2812 PIO Driver](https://github.com/ithinuel/ws2812-pio-rs)
* [Adafruit Feather RP2040 Schematic](https://learn.adafruit.com/assets/100337)
* [Adafruit Feather RP2040 Pinout](https://learn.adafruit.com/assets/105204)
* [RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
* [Othneil Drew's Best README Template](https://github.com/othneildrew/Best-README-Template)

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->

[stars-shield]: https://img.shields.io/github/stars/jstevej/rusty-feather?style=for-the-badge
[stars-url]: https://github.com/jstevej/rusty-feather/stargazers
[forks-shield]: https://img.shields.io/github/forks/jstevej/rusty-feather?style=for-the-badge
[forks-url]: https://github.com/jstevej/rusty-feather/network/members
[license-shield]: https://img.shields.io/github/license/jstevej/rusty-feather?style=for-the-badge
[license-url]: https://github.com/jstevej/rusty-feather/blob/main/LICENSE.txt
[product-screenshot]: images/screenshot.png
