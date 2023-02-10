# Roster Generator

This is a tool to generate a document that contains information about all ROS objects in your system.

It is a python script that looks through source files and parses topic and service declarations, then formats and adds them to a file.

To run it, put `generate.py` in an empty directory along with a folder containing all your source files. The generator works with both Python and C++ files. The directory structure should look like this:

```
\<parent directory\>

+--generate.py

+--\<source directory\>

+-----(your source files)
```

Keep in mind that the parser is not magic and often can't deal with any deviations you make from the standard format. Keeping your declarations consistent is up to you.

The output is in `roster.txt`. Each declaration is a line in this file that tells you the message type, the ROS path, the endpoint type (publisher, client, etc.), and what file to find it in. If an identical line already exists in the file, a new one will not be added. This allows you to add comments to the output file to document the function of each object. 
