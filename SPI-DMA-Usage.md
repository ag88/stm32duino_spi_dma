# stm32duino_spi_dma — Quick Usage Guide

This document explains how to use the stm32duino_spi_dma SPI library, with emphasis on the DMA-enabled bulk transfer APIs.

Summary
- Include the library with:
  #include "SPI.h"
- The library exposes a global `SPI` object for the default SPI peripheral (usually SPI1) and types named `SPIClass1`, `SPIClass2`, `SPIClass3`. Each of those types is bound to the corresponding hardware peripheral:
  - `SPIClass1` → SPI1
  - `SPIClass2` → SPI2
  - `SPIClass3` → SPI3 (only defined when the MCU supports SPI3)
- On each STM32 series the concrete class behind these typedefs is provided by a series-specific header (e.g. `SPIDMA_F4XX.h`, `SPIDMA_G4XX.h`, …). The `SPI` symbol (the commonly used default) is an `extern` instance created by those headers for the default peripheral.
- DMA-enabled functions:
  - `virtual void transfer(void *buf, size_t count, bool skipReceive = SPI_TRANSMITRECEIVE)` — single-buffer bulk transfer. When `skipReceive == true` the transfer is asynchronous (the call returns while DMA is sending).
  - `virtual void transfer(const void *tx_buf, void *rx_buf, size_t count)` — two-buffer bulk transfer (blocking). This call waits for DMA to finish.
  - `virtual void transfer_async(const void *tx_buf, void *rx_buf, size_t count)` — two-buffer transfer returned immediately (asynchronous).
  - `virtual bool isTransferComplete()` — use to poll whether the (asynchronous) DMA transfer has finished.

Important notes up front
- Always call `begin()` (and usually `beginTransaction(SPISettings)`) before using the transfer APIs.
- For asynchronous DMA transfers, the memory pointed to by `tx_buf` (and by `buf` for single-buffer async transmit-only) must remain valid and untouched until `isTransferComplete()` reports true.
- The concrete semantics of which core clock is used for baud-rate calculations (PCLK1 vs PCLK2) are handled by the concrete series class implementations of `getClkFreq()` — you do not need to call `getClkFreq()` yourself but you should be aware that SPI clock selection is MCU-specific.
- `transfer8`/`transfer16` semantics follow the `SPIClass` contract. `transfer16()` has a default implementation that delegates to the byte-wise transfer if not overridden.

Basic API overview

- Initialization
  - `SPI.begin()` — initialize hardware, pins, DMA and NVIC hooks for the configured peripheral.
  - `SPI.end()` — de-initialize (defined in base class).
  - `SPI.beginTransaction(SPISettings settings)` / `SPI.endTransaction()` — lock settings for critical regions.

- Single-byte transfers
  - `uint8_t transfer(uint8_t data, bool skipReceive = SPI_TRANSMITRECEIVE)` — send one byte; can optionally skip receive (transmit-only).

- Bulk transfers
  - `void transfer(void *buf, size_t count, bool skipReceive = SPI_TRANSMITRECEIVE)`
    - Single-buffer mode: TX and RX share the same buffer; RX bytes overwrite the buffer contents.
    - If `skipReceive == true`, this function starts a DMA transmit-only operation and returns immediately (async). Use `isTransferComplete()` to check.
    - If `skipReceive == false`, the function performs a full duplex transfer. Implementation may be synchronous (wait for completion) or use DMA; check series implementation. For this library the buffer versions are DMA-enabled.
  - `void transfer(const void *tx_buf, void *rx_buf, size_t count)`
    - Two-buffer blocking transfer. This will wait until DMA completes and return only when finished.
  - `void transfer_async(const void *tx_buf, void *rx_buf, size_t count)`
    - Two-buffer non-blocking DMA transfer. Returns immediately; call `isTransferComplete()` to check completion.
  - `bool isTransferComplete()`
    - Query whether the last asynchronous transfer has finished. Use this to safely reuse buffers or to proceed after a non-blocking transfer.

Example sketches

1) Simple single-byte use (typical Arduino style)
```cpp
#include "SPI.h"

void setup() {
  pinMode(PA4, OUTPUT); // example chip select pin
  digitalWrite(PA4, HIGH);

  SPI.begin(); // initialize default SPI (usually SPI1)
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // 1 MHz
}

void loop() {
  digitalWrite(PA4, LOW);
  uint8_t reply = SPI.transfer(0x9A); // send a byte and receive the reply
  digitalWrite(PA4, HIGH);

  delay(1000);
}
```

2) Single-buffer DMA in-place full-duplex transfer (blocking or async transmit-only)
```cpp
#include "SPI.h"

// Example: in-place read/write of 256 bytes.
// If you want to only transmit and not receive, pass skipReceive = SPI_TRANSMITONLY (true).

uint8_t buf[256];

void setup() {
  SPI.begin();
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0)); // 8 MHz
  // prepare buf with bytes to send
  for (size_t i = 0; i < sizeof(buf); ++i) buf[i] = i;
}

void loop() {
  // Full duplex, receive overwrites the buffer (blocking or DMA-managed)
  SPI.transfer(buf, sizeof(buf), SPI_TRANSMITRECEIVE); // skipReceive == false

  // ... buf now contains received bytes ...

  delay(1000);
}
```

Send-only (async transmit-only) with single-buffer:
```cpp
#include "SPI.h"
uint8_t txbuf[512];

void setup() {
  SPI.begin();
  // fill txbuf...
}

void loop() {
  // Start transmit-only DMA and return immediately:
  SPI.transfer(txbuf, sizeof(txbuf), SPI_TRANSMITONLY); // skipReceive == true

  // Do other work here while DMA sends.
  while (!SPI.isTransferComplete()) {
    // optionally yield or do other tasks
  }

  // txbuf can now be reused
  delay(500);
}
```

3) Two-buffer blocking DMA transfer (full duplex)
```cpp
#include "SPI.h"

uint8_t txbuf[128];
uint8_t rxbuf[128];

void setup() {
  SPI.begin();
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  // fill txbuf...
}

void loop() {
  // This call waits until DMA completes and rxbuf contains the received bytes.
  SPI.transfer(txbuf, rxbuf, sizeof(txbuf));

  // process rxbuf...

  delay(1000);
}
```

4) Two-buffer asynchronous DMA transfer
```cpp
#include "SPI.h"

uint8_t txbuf[256];
uint8_t rxbuf[256];

void setup() {
  SPI.begin();
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
}

void loop() {
  // Start transfer and return immediately
  SPI.transfer_async(txbuf, rxbuf, sizeof(txbuf));

  // Do other tasks...
  // Poll for completion (or use a synchronization mechanism inside your app)
  while (!SPI.isTransferComplete()) {
    // do useful work
  }

  // Now rxbuf is valid
  process(rxbuf);
}
```

Using different SPI peripherals (SPI2/SPI3)
- The library provides the typedefs `SPIClass1`, `SPIClass2`, `SPIClass3` which are bound to the underlying series-specific classes for SPI1/2/3. You can create objects for other SPI peripherals explicitly:

```cpp
#include "SPI.h"

// use the default `SPI` instance (usually SPI1)
extern SPIClass1 SPI; // the library headers already declare `extern SPI;`

// If you need to instantiate/operate SPI2 or SPI3 directly:
SPIClass2 SPI2; // SPI2 object bound to the SPI2 peripheral
// If SPI3 is supported on your MCU:
#if defined(SPI3_BASE)
SPIClass3 SPI3;
#endif

void setup() {
  SPI2.begin();
  SPI2.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  // ...
}
```

(Notes: On some targets `SPIClass3` may not be defined if the MCU has no SPI3 hardware. Also some headers expose an `extern SPI` instance for the default peripheral — consult the MCU-specific `SPIDMA_*` header if you need the exact symbol names.)

Advanced and implementation details
- The DMA-enabled class (`SPI_DMA` / series-specific subclasses) overrides the buffer-oriented transfer methods so transfers use HAL DMA primitives when possible.
- The base `SPIClass` exposes `getHandle()` to access the `SPI_HandleTypeDef` used by HAL; you can use it if you need to combine HAL-level code with the library (use at your own risk).
- Derived classes must implement `initSPI()`, `initDMA()`, `initNVIC()`, and `getClkFreq()`; these are provided in series-specific files so the library adapts to the STM32 series.
- `SPISettings` is available to specify `clock`, `bitOrder` and `dataMode`. Example: `SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));`

Best practices and common pitfalls
- For asynchronous DMA transfers:
  - Never modify or free the buffers passed to an async transfer until `isTransferComplete()` returns true.
  - Do not start another DMA transfer on the same SPI instance before the previous one finishes (or ensure your code sequences transfers appropriately).
- If you need precise timing or lower latency, tune the SPI clock via `SPISettings` and verify the derived baud rate for your MCU’s clock tree (the concrete `getClkFreq()` implementation on the target decides which peripheral clock is used).
- If the peripheral uses specific pins (alternate functions), make sure to pass correct pin numbers to the constructor or use the default board-defined MOSI/MISO/SCLK/SSEL values.

Where to look in source
- `SPI.h` — provides `extern SPI` and typedefs `SPIClass1/2/3` that map to the series-specific implementations.
- `SPIDMA.h` — defines the DMA-enabled SPI class and documents the DMA-specific transfer functions and semantics.
- `SPIClass.h` — base, Arduino-compatible SPI API; documents the abstract functions and `SPISettings`.

If you need an example for your specific STM32 family, look at the series-specific files (e.g. `SPIDMA_F4XX.h`, `SPIDMA_G4XX.h`, etc.) in the `src/SPI/` folder — they show the peripheral-to-class mappings and initialization details for that series.

License and credits
- Portions of the SPI API originate from the Arduino Core for STM32 (stm32duino) and other contributors. See headers in the source for copyright/credit information.

---

This guide should help you get started with the DMA-enabled bulk transfer functions and the mapping between `SPIClass1/2/3` and the physical SPI peripherals. 
