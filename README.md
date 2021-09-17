# USB device to hardware UART for ATSAMD chips

This isn't ready for public use yet.
Only supports 8 bit character size.

## TODO
- Update dependency on atsamd-rs to current
- Make an example file
- Use DMA to feed the SERCOM, this ISR-based approach only gets about 50% of
  theoretical throughput
