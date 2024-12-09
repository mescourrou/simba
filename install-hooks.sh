#!/bin/bash

cd .git/hooks
if [ ! -f pre-commit ]; then
    cat << 'EOF' > pre-commit
#!/bin/bash

for line in $(git status -s)
do
    # if added or modified
    if [[ $line == A* || $line == M* ]]
    then
        # check file extension
        if [[ $line == *.rs ]]
        then
            # format file
            rustfmt $(pwd)/${line:3}
            # add changes
            git add $(pwd)/${line:3}
        fi
    fi
done

EOF
chmod +x pre-commit
fi