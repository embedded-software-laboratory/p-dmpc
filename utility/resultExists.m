function exists = resultExists(options)

    arguments
        options (1, 1) Config;
    end

    exists = ~isempty(load_all(options));
end
