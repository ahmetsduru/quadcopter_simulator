function actual_y_t(log_folder)
    % Dosya yolunu belirleme (log klasöründen okuma)
    file_path = fullfile(log_folder, 'states.txt');

    % Veriyi okuma (readtable kullanarak daha esnek şekilde okuma)
    data = readtable(file_path, 'Delimiter', ',');

    % Timestamp (1. kolon) ve actual_y (2. kolon) verilerini ayırma
    timestamp = data{:, 1};  % 1. kolon: timestamp
    actual_y = data{:, 3};      % 3. kolon: actual_y

    % Eğer veriler doğruysa, grafiği çizelim
    if ~isempty(timestamp) && ~isempty(actual_y)
        figure;
        plot(timestamp, actual_y, 'LineWidth', 1.5);
        xlabel('Timestamp (s)');
        ylabel('actual_y');
        title('actual_y vs. Timestamp');
        grid on;
    else
        disp('Veriler boş olabilir, lütfen dosya formatını kontrol edin.');
    end
end