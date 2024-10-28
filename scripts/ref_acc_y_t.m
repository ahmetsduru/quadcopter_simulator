function ref_acc_y_t(log_folder)
    % Dosya yolunu belirleme (log klasöründen okuma)
    file_path = fullfile(log_folder, 'ref_pose.txt');

    % Veriyi okuma (readtable kullanarak daha esnek şekilde okuma)
    data = readtable(file_path, 'Delimiter', ',');

    % Timestamp (1. kolon) ve ref_acc_y (9. kolon) verilerini ayırma
    timestamp = data{:, 1};  % 1. kolon: timestamp
    ref_acc_y = data{:, 9};      % 9. kolon: ref_acc_y

    % Eğer veriler doğruysa, grafiği çizelim
    if ~isempty(timestamp) && ~isempty(ref_acc_y)
        figure;
        plot(timestamp, ref_acc_y, 'LineWidth', 1.5);
        xlabel('Timestamp (s)');
        ylabel('ref_acc_y');
        title('ref_acc_y vs. Timestamp');
        grid on;
    else
        disp('Veriler boş olabilir, lütfen dosya formatını kontrol edin.');
    end
end