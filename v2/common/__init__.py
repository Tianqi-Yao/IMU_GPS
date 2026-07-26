"""common — shared, zero-business-logic infrastructure for all bridge modules.

Only framework code belongs here: WebSocket broadcast plumbing, static-file
HTTP serving, logging bootstrap, and the HTTP->WebSocket port convention.
No module-specific parsing, filtering, or control logic may live in this
package; the dependency direction is one-way, from 00_QR..06_Camera into
common/, never the reverse.
"""
