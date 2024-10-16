function actual_x_t(log_folder)
    % Dosya yolunu belirleme (log klasöründen okuma)
    file_path = fullfile(log_folder, 'states.txt');

    % Veriyi okuma (readtable kullanarak daha esnek şekilde okuma)
    data = readtable(file_path, 'Delimiter', ',');

    % Timestamp (1. kolon) ve actual_x (2. kolon) verilerini ayırma
    timestamp = data{:, 1};  % 1. kolon: timestamp
    actual_x = data{:, 2};      % 2. kolon: actual_x

    % Eğer veriler doğruysa, grafiği çizelim
    if ~isempty(timestamp) && ~isempty(actual_x)
        figure;
        plot(timestamp, actual_x, 'LineWidth', 1.5);
        xlabel('Timestamp (s)');
        ylabel('actual_x');
        title('actual_x vs. Timestamp');
        grid on;
    else
        disp('Veriler boş olabilir, lütfen dosya formatını kontrol edin.');
    end
end






