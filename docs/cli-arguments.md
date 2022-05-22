# how to create avidrone cli arguments

>for an in-depth tutorial, see the [source](https://dbader.org/blog/how-to-make-command-line-commands-with-python)

## step 1: Mark your Python file as executable

On terminal, mark your script as executable

```{bash}
chmod +x example.py
```

## Step 2: Add an interpreter "shebang"

Add this to the top of your python script

```{bash}
#!/usr/bin/env python3
```

```{bash}
mv example.py example
```

## Step 3: Make sure your program is on the PATH

Create the ~/bin directory:

```{bash}
mkdir -p ~/bin
```

Next, copy your script of ~/bin

```{bash}
cp example ~/bin
```

Finally, add ~/bin to your PATH:

```(bash)
export PATH=$PATH":$HOME/bin"
```
