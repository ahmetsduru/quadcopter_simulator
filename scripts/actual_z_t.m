function actual_z_t(log_folder)
    % Dosya yolunu belirleme (log klasöründen okuma)
    file_path = fullfile(log_folder, 'states.txt');

    % Veriyi okuma (readtable kullanarak daha esnek şekilde okuma)
    data = readtable(file_path, 'Delimiter', ',');

    % Timestamp (1. kolon) ve actual_z (4. kolon) verilerini ayırma
    timestamp = data{:, 1};  % 1. kolon: timestamp
    actual_z = data{:, 4};      % 4. kolon: actual_z

    % Eğer veriler doğruysa, grafiği çizelim
    if ~isempty(timestamp) && ~isempty(actual_z)
        figure;
        plot(timestamp, actual_z, 'LineWidth', 1.5);
        xlabel('Timestamp (s)');
        ylabel('actual_z');
        title('actual_z vs. Timestamp');
        grid on;
    else
        disp('Veriler boş olabilir, lütfen dosya formatını kontrol edin.');
    end
end