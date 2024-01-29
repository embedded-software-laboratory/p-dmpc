function experiment_result = repeat()
    options = Config.load_from_file('Config.json');
    experiment_result = main(options);
end

