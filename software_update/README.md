# Fixposition Software Update

Command line software update support.

## Python

This Python script uploads the software image to the system and then waits for
the notification of success or failure of the flash process.

### Dependencies

-  [Python3](https://www.python.org/download/releases/3.0/)
-  [Python3 - asyncio](https://docs.python.org/3/library/asyncio.html)
-  [Python3 - json](https://docs.python.org/3/library/json.html)
-  [Python3 - requests](https://requests.readthedocs.io/en/latest/)
-  [Python3 - websockets](https://websockets.readthedocs.io/en/stable/index.html)
-  [Python3 - argparse](https://docs.python.org/3/library/argparse.html)

### Usage

```sh
$ ./software_update.py -h
usage: software_update.py [-h] path ip timeout

positional arguments:
  path
  ip
  timeout

optional arguments:
  -h, --help  show this help message and exit
```

The software update process will take several minutes.

### Example

```sh
$ python3 ./software_update.py image.swu 10.0.2.1 600
Start uploading image...

Software image uploaded successfully. Wait for installation to be finished...

Wait update finished
Software update successful !
```

## Curl

The HTTP upload API can also be used directly with `curl`. In this case only the
software image is uploaded and a response about success or failure of the flash
process is not waited or checked.

### Example (For software version 2.63 or newer - after 09.03.2023)

```sh
curl -v -F name=@image.swu http://10.0.2.1/update/upload
*   Trying 10.0.2.1:8080...
* TCP_NODELAY set
* Connected to 10.0.2.1 (10.0.2.1) port 80 (#0)
> POST /update/upload HTTP/1.1
> Host: 10.0.2.1
> User-Agent: curl/7.66.0
> Accept: */*
> Content-Length: 435603202
> Content-Type: multipart/form-data; boundary=------------------------5ee14ea9339300b5
> Expect: 100-continue
>
* Done waiting for 100-continue
* We are completely uploaded and fine
* Mark bundle as not supporting multiuse
< HTTP/1.1 200 OK
< Server: Mongoose/6.18
< Content-Type: text/plain
< Connection: close
<
Ok, image.swu - 435602944 bytes.
```

### Example (For software version 2.58.2 or older - before 17.01.2023)

```sh
curl -v -F name=@image.swu http://10.0.2.1:8080/upload
*   Trying 10.0.2.1:8080...
* TCP_NODELAY set
* Connected to 10.0.2.1 (10.0.2.1) port 8080 (#0)
> POST /upload HTTP/1.1
> Host: 10.0.2.1:8080
> User-Agent: curl/7.66.0
> Accept: */*
> Content-Length: 435603202
> Content-Type: multipart/form-data; boundary=------------------------5ee14ea9339300b5
> Expect: 100-continue
>
* Done waiting for 100-continue
* We are completely uploaded and fine
* Mark bundle as not supporting multiuse
< HTTP/1.1 200 OK
< Server: Mongoose/6.18
< Content-Type: text/plain
< Connection: close
<
Ok, image.swu - 435602944 bytes.
```


## License
[software_update.py](software_update.py) is licensed under `GNU General Public License v3.0 or later` - see the [LICENSE](LICENSE) file for details
