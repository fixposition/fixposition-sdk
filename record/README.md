# Fixposition Recording Utilities

Recording over network (ethernet) for Vision-RTK 2, software version 2.85.3 or 2.90.0 and later.

---
## record.sh

A minimal demonstration for using the recording API. There is no error handling.

### Dependencies

- Linux
- Bash
- curl

### Usage

```sh
./record.sh 10.0.2.1 medium
```

---
## record.py

A more sophisticated script with error handling and status reporting.

### Dependencies

- Linux
- Python 3 (tested with 3.10)

### Usage

```sh
./record.py 10.0.2.1 medium
```

---
## License

MIT, see [LICENSE](LICENSE)
