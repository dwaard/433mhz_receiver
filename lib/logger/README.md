# EmbeddedLogger

Reusable, channel-based logger for PlatformIO and Arduino projects.

## Folder layout

- `Logger.h/.cpp`: logger core and global `Log` instance
- `LogTypes.h`: shared log levels and payload code constants
- `LogEvent.h`: event payload passed to channels
- `LogChannel.h/.cpp`: base channel and formatting helpers
- `Channels/`: optional channel implementations (serial, telnet, callback)
- `TimestampProvider.h`: timestamp abstraction
- `TimestampProviders/`: concrete timestamp providers

## Ownership model

`Logger` now supports two registration methods:

- `addChannel(LogChannel*)`: non-owning, for stack or externally owned channels
- `addOwnedChannel(LogChannel*)`: logger owns and deletes the channel

Use `addOwnedChannel` for channels created with `new`.

## Example

```cpp
#include <Arduino.h>
#include "Logger.h"
#include "Channels/SerialChannel.h"

void setup() {
  Serial.begin(115200);
  Log.addOwnedChannel(new SerialChannel(Logger::DEBUG));
  Log.info("logger ready");
}
```

## Notes for reuse

- Keep include path casing exactly as folder names (`Channels/...`).
- Prefer `LoggerTypes` constants for lower coupling in reusable components.
