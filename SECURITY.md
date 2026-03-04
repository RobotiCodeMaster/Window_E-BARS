# Security Policy

## Supported Versions | Arduino IDE

Based on the libraries and APIs used in the code (WiFi.h, time.h, EEPROM.h, ESP32Servo.h, etc.),
the sketch supports all modern Arduino IDE versions that support the ESP32 core.

| Version | Supported          |
| ------- | ------------------ |
| 1.8.19  | :white_check_mark: |
| 2.0.x   | :white_check_mark: |
| 2.1.x   | :white_check_mark: |
| 2.2.x   | :white_check_mark: |
| 2.3.x   | :white_check_mark: |

## Supported Microcontrollers

	•	Arduino ESP32 Core ≥ 2.0.11
	•	Works best with 2.0.14+
  
## Reporting a Vulnerability

**Security & Reliability Guidelines for Contributors**

The following practices must be followed when modifying or extending the system. These rules are intended to preserve system reliability, safety behavior, and data integrity.

- Serial Command Interface Control

The command-line interface (CLI) can modify safety-critical parameters (servo positions, thresholds, simulation mode, and log operations). To prevent misuse or unintended configuration changes, implement controlled access.

Recommended approaches:
	•	Compile-time control
	•	Provide a build flag such as ENABLE_CLI.
	•	Production builds should compile with CLI disabled.
	•	Physical configuration enable
	•	Only allow CLI commands if a designated configuration jumper pin is held LOW during boot.
	•	Startup time window
	•	Accept commands only within a limited time window (for example, the first 60 seconds after boot).
	•	Input rate limiting
	•	Ignore excessive serial input to prevent command flooding.

These controls ensure the device cannot be reconfigured unintentionally during normal operation.


- Event Log Integrity

The event log is intended to preserve a reliable timeline of system behavior. The following protections help ensure logs remain trustworthy:
	•	Add a CRC (CRC16 or CRC32) to both StatsData and EventRecord.
	•	Reject records that fail CRC validation when reading logs.
	•	Maintain a monotonic event counter so events cannot be silently reordered.
	•	Maintain a last valid write index marker to allow safe recovery after power loss.
	•	Provide an optional read-only mode preventing log deletion without a physical hardware jumper.

These protections allow the system to detect corrupted or partially written records.


- Statistics Storage Protection

Statistics stored in persistent memory should be protected against corruption caused by unexpected power loss.

Recommended method:
	•	Store statistics using a dual-slot (A/B) structure.
	•	Each slot contains:
	•	Version counter
	•	CRC
	•	Statistics data
	•	On write:
	•	Write the next slot with incremented version and valid CRC.
	•	On boot:
	•	Load the slot with the highest valid version number.

This prevents a single interrupted write from destroying stored statistics.


- Persistent Storage Write Strategy

Frequent writes reduce storage lifespan and can stall the system during blocking operations.

Implement the following policies:
	•	Buffer log writes
	•	Accumulate several events before committing them to storage.
	•	Timed flushing
	•	Flush buffered events at regular intervals.
	•	Immediate flush for critical events
	•	Fire detection
	•	Rapid temperature rise
	•	System faults
	•	Avoid repeated writes of identical data
	•	Only update statistics when values actually change.

This reduces unnecessary wear on persistent memory while preserving critical data.


- Safety Interlocks for Deployment

To ensure reliable operation in real environments, deploy the following safety mechanisms where possible:
	•	Dual-condition deployment logic (optional configuration)
	•	Deployment can require multiple indicators such as:
	•	Fire threshold detection
	•	Rapid temperature rise
	•	Confirmation delay
	•	Servo movement verification
	•	Monitor time required to reach the commanded position.
	•	If movement does not complete within expected time, log a servo fault event.
	•	Tamper detection (optional hardware feature)
	•	Add an enclosure switch or similar mechanism.
	•	Log tamper events when the system housing is opened.

These mechanisms reduce the risk of accidental or false system activation.


- Debug Message Standards

All debug and diagnostic output must remain professional and neutral.

Guidelines:
	•	Avoid offensive or hostile language in messages.
	•	Use clear and descriptive text indicating system state or error condition.
	•	Messages should be suitable for review in logs, presentations, and shared repositories.


- Power-Loss Resilience

The system must be able to recover safely from sudden power interruptions.

To support this:
	•	Validate all persistent records with CRC.
	•	Ensure writes occur in deterministic steps.
	•	Ensure partially written records are ignored on boot.
	•	Maintain a reliable mechanism to determine the last valid log entry.

This guarantees the system always boots into a consistent state.


Following these practices ensures the system remains reliable, verifiable, and safe when deployed or extended by other contributors.
