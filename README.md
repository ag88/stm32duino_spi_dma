# stm32duino_spi_dma
SPI with DMA for [stm32duino](https://github.com/stm32duino/Arduino_Core_STM32) &nbsp; [Wiki](https://github.com/stm32duino/Arduino_Core_STM32/wiki)

## This is not an official implementation

## This is preliminary / experimental / alpha / untested codes

This is preliminary / experimental / alpha / untested codes still being changed.

## Use

To use the library e.g.:
```
#include "SPI.h"

SPIClass1 spi1;
SPIClass2 spi2;
SPIClass3 spi3;

uint32_t spifreq = 10000000; //10 Mhz
SPISettings spisettings(spifreq, BitOrder::MSBFIRST, SPIMode::SPI_MODE0);

#define BUFSIZE 256
uint8_t sendbuf[BUFSIZE], recvbuf[BUFSIZE];

void setup() {
spi1.begin(spisettings);
spi2.begin(spisettings);
spi3.begin(spisettings);

uint8_t data = 1;
memset(sendbuf, data, BUFSIZE);
memset(recvbuf, 0, BUFSIZE);
}

void loop() {
uint8_t data = 1, read;
read = spi1.transfer(data);
read = spi2.transfer(data);
read = spi3.transfer(data);

spi1.transfer(recvbuf, BUFSIZE);
spi2.transfer(recvbuf, BUFSIZE);
spi3.transfer(recvbuf, BUFSIZE);

spi1.transfer(sendbuf, recvbuf, BUFSIZE);
spi2.transfer(sendbuf, recvbuf, BUFSIZE);
spi3.transfer(sendbuf, recvbuf, BUFSIZE);

delay(1);
}
```
- Each SPIClass1, SPIClass2, SPIClass3 connects to the specific SPI peripheral 1, 2, 3 respectively
- Each SPIClassN is actually derived from SPIClass, hence they can be passed to functions / methods
that use SPIClass for the SPI api. see [SPIClass.h](src/SPI/SPIClass.h).

### No Warranties

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

### Forum
The community forum is https://www.stm32duino.com

Original thread is https://www.stm32duino.com/viewtopic.php?t=2553

