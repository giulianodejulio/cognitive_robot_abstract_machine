# %% compact terminal view; original logs remain on disk
/^Unknown (attribute|tag) / { next }
/^\[INFO\]/ { next }
/^INFO:/ { next }
/^WARNING:.*Model reload synchronization not available/ { next }
/^Traceback \(most recent call last\):/ { traceback = 1; next }
traceback && /^[[:space:]]/ { next }
/^(KeyboardInterrupt|During handling of the above exception|The above exception was the direct cause)/ { next }
/^SENDING:/ { onset = 0 }
/^WHEEL ONSET:/ { onset = 1 }
/^REQUEST TO WHEEL FEEDBACK:/ && onset { next }
{ traceback = 0; print; fflush() }
